//! monstertruck-draw: DRAW-artige Kommando-Shell.
//! Aufruf: `draw skript.dr [...]` oder interaktiv über stdin.
//! Alle Längen in mm, Winkel in Grad. Intern Meter (Massstabslektion).

use anyhow::{Result, anyhow};
use monstertruck_io::step::load::Table;
use monstertruck_io::step::save::{CompleteStepDisplay, StepModel};
use monstertruck_meshing::prelude::*;
use monstertruck_modeling::*;
use std::collections::BTreeMap;
use std::io::{BufRead, Write};
use std::time::Instant;

const FINE: f64 = 0.0005; // Netztoleranz (m) für vol/check/dump
const SWEEP_MM: [f64; 8] = [0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 4.0, 8.0];
const K: f64 = 1000.0;

struct App {
    objs: BTreeMap<String, Solid>,
}

// ---------- Geometrie-Helfer (intern Meter) ----------

fn boxsolid(sx: f64, sy: f64, sz: f64) -> Solid {
    let v = builder::vertex(Point3::origin());
    let e = builder::extrude(&v, Vector3::unit_x() * sx);
    let f = builder::extrude(&e, Vector3::unit_y() * sy);
    builder::extrude(&f, Vector3::unit_z() * sz)
}

fn frustum(r0: f64, r1: f64, h: f64) -> Result<Solid> {
    let w0: Wire = primitive::circle(
        Point3::new(r0, 0.0, 0.0),
        Point3::origin(),
        Vector3::unit_z(),
        4,
    );
    let w1: Wire = primitive::circle(
        Point3::new(r1, 0.0, h),
        Point3::new(0.0, 0.0, h),
        Vector3::unit_z(),
        4,
    );
    let mut sh: Shell =
        builder::try_skin_wires(&[w0.clone(), w1.clone()]).map_err(|e| anyhow!("Mantel: {e:?}"))?;
    sh.push(builder::try_attach_plane(&[w0.inverse()]).map_err(|e| anyhow!("Boden: {e:?}"))?);
    sh.push(builder::try_attach_plane(&[w1]).map_err(|e| anyhow!("Deckel: {e:?}"))?);
    Solid::try_new(vec![sh]).map_err(|e| anyhow!("nicht geschlossen: {e:?}"))
}


/// Übergangsstück 300x200x260 mm: Grundriss-Rechteck mit Eckradius `re_mm`,
/// Deckel Kreis Ø150 um (0, 25). Masse fest, Eckradius frei — der Radius
/// entscheidet, ob die Eckflächen boolesch-tauglich sind (>=1 mm) oder in
/// eine Singularität laufen (0 geht konstruktiv nicht, Minimum 0.1 mm).
fn uebergang(re_mm: f64) -> Result<Solid> {
    if re_mm < 0.1 {
        return Err(anyhow!("Eckradius >= 0.1 mm nötig (Singularität)"));
    }
    let re = re_mm / K;
    let (hx, hy, h, rk, cy) = (0.150, 0.100, 0.260, 0.075, 0.025);
    let v = |x: f64, y: f64, z: f64| builder::vertex(Point3::new(x, y, z));
    let a1 = v(-hx + re, -hy, 0.0);
    let a2 = v(hx - re, -hy, 0.0);
    let b1 = v(hx, -hy + re, 0.0);
    let b2 = v(hx, hy - re, 0.0);
    let c1 = v(hx - re, hy, 0.0);
    let c2 = v(-hx + re, hy, 0.0);
    let d1 = v(-hx, hy - re, 0.0);
    let d2 = v(-hx, -hy + re, 0.0);
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let ecke = |cx: f64, cyy: f64, sx: f64, sy: f64| Point3::new(cx + re * s * sx, cyy + re * s * sy, 0.0);
    let top = |grad: f64| {
        let t = (grad as f64).to_radians();
        Point3::new(rk * t.cos(), cy + rk * t.sin(), h)
    };
    let winkel = [-95.0, -85.0, -5.0, 5.0, 85.0, 95.0, 175.0, 185.0];
    let tv: Vec<_> = winkel.iter().map(|g| builder::vertex(top(*g))).collect();
    let tp = |g0: f64, g1: f64| top((g0 + g1) / 2.0);
    let unten: Wire = vec![
        builder::line(&a1, &a2),
        builder::circle_arc(&a2, &b1, ecke(hx - re, -hy + re, 1.0, -1.0)),
        builder::line(&b1, &b2),
        builder::circle_arc(&b2, &c1, ecke(hx - re, hy - re, 1.0, 1.0)),
        builder::line(&c1, &c2),
        builder::circle_arc(&c2, &d1, ecke(-hx + re, hy - re, -1.0, 1.0)),
        builder::line(&d1, &d2),
        builder::circle_arc(&d2, &a1, ecke(-hx + re, -hy + re, -1.0, -1.0)),
    ]
    .into();
    let mut oben_e = Vec::new();
    for i in 0..8 {
        let j = (i + 1) % 8;
        let g1 = if j == 0 { 265.0 } else { winkel[j] };
        oben_e.push(builder::circle_arc(&tv[i], &tv[j], tp(winkel[i], g1)));
    }
    let oben: Wire = oben_e.into();
    let mut shell: Shell = builder::try_skin_wires(&[unten.clone(), oben.clone()])
        .map_err(|e| anyhow!("Mantel: {e:?}"))?;
    shell.push(builder::try_attach_plane(&[unten.inverse()]).map_err(|e| anyhow!("Boden: {e:?}"))?);
    shell.push(builder::try_attach_plane(&[oben]).map_err(|e| anyhow!("Deckel: {e:?}"))?);
    Solid::try_new(vec![shell]).map_err(|e| anyhow!("nicht geschlossen: {e:?}"))
}

fn mesh_of(s: &Solid) -> PolygonMesh {
    s.triangulation(FINE).to_polygon()
}

fn bbox(m: &PolygonMesh) -> (Point3, Point3) {
    let mut lo = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut hi = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for p in m.positions() {
        lo = Point3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Point3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    (lo, hi)
}

fn naked_edges(m: &PolygonMesh) -> Vec<(usize, usize)> {
    // truck-Netze teilen Vertices nicht über Flächengrenzen: erst verschweissen.
    let eps = 1.0e-7;
    let mut weld: std::collections::HashMap<(i64, i64, i64), usize> = Default::default();
    let canon: Vec<usize> = m
        .positions()
        .iter()
        .map(|p| {
            let key = ((p.x / eps).round() as i64, (p.y / eps).round() as i64, (p.z / eps).round() as i64);
            let next = weld.len();
            *weld.entry(key).or_insert(next)
        })
        .collect();
    let mut cnt: std::collections::HashMap<(usize, usize), u32> = Default::default();
    for t in m.faces().triangle_iter() {
        let idx = [canon[t[0].pos], canon[t[1].pos], canon[t[2].pos]];
        for k in 0..3 {
            let (a, b) = (idx[k], idx[(k + 1) % 3]);
            if a != b {
                *cnt.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
    }
    cnt.into_iter().filter(|&(_, n)| n == 1).map(|(e, _)| e).collect()
}

// ---------- BMP-Rasterizer (dump: kein GPU, kein Fenster) ----------

fn write_bmp(path: &str, w: usize, h: usize, rgb: &[u8]) -> Result<()> {
    let row = (w * 3 + 3) & !3;
    let data = row * h;
    let mut f = std::fs::File::create(path)?;
    let mut hdr = Vec::with_capacity(54);
    hdr.extend(b"BM");
    hdr.extend(&(54u32 + data as u32).to_le_bytes());
    hdr.extend(&[0u8; 4]);
    hdr.extend(&54u32.to_le_bytes());
    hdr.extend(&40u32.to_le_bytes());
    hdr.extend(&(w as i32).to_le_bytes());
    hdr.extend(&(h as i32).to_le_bytes());
    hdr.extend(&1u16.to_le_bytes());
    hdr.extend(&24u16.to_le_bytes());
    hdr.extend(&[0u8; 24]);
    f.write_all(&hdr)?;
    let pad = vec![0u8; row - w * 3];
    for y in (0..h).rev() {
        for x in 0..w {
            let i = (y * w + x) * 3;
            f.write_all(&[rgb[i + 2], rgb[i + 1], rgb[i]])?; // BGR
        }
        f.write_all(&pad)?;
    }
    Ok(())
}

fn dump(m: &PolygonMesh, preset: &str, px: usize, path: &str) -> Result<()> {
    let (w, h) = (px, px * 3 / 4);
    let pts = m.positions();
    let c = {
        let (lo, hi) = bbox(m);
        Point3::new((lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0, (lo.z + hi.z) / 2.0)
    };
    let (sa, ca) = (-(62f64.to_radians())).sin_cos();
    let (sb, cb) = (32f64.to_radians()).sin_cos();
    let proj = |p: &Point3| -> [f64; 3] {
        let (x, y, z) = (p.x - c.x, p.y - c.y, p.z - c.z);
        match preset {
            "front" => [x, z, -y],
            "top" => [x, y, z],
            _ => {
                let (rx, ry) = (x * cb - y * sb, x * sb + y * cb); // Rz
                [rx, ry * ca - z * sa, ry * sa + z * ca] // Rx, depth = Zeile 3
            }
        }
    };
    let q: Vec<[f64; 3]> = pts.iter().map(|p| proj(p)).collect();
    let (mut xmin, mut xmax, mut ymin, mut ymax) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for p in &q {
        xmin = xmin.min(p[0]);
        xmax = xmax.max(p[0]);
        ymin = ymin.min(p[1]);
        ymax = ymax.max(p[1]);
    }
    let s = 0.9 * (w as f64 / (xmax - xmin)).min(h as f64 / (ymax - ymin));
    let light = {
        let l = [0.4f64, 0.35, 0.85];
        let n = (l[0] * l[0] + l[1] * l[1] + l[2] * l[2]).sqrt();
        [l[0] / n, l[1] / n, l[2] / n]
    };
    let mut img = vec![245u8; w * h * 3];
    let mut zb = vec![f64::MIN; w * h];
    for t in m.faces().triangle_iter() {
        let v: Vec<[f64; 3]> = (0..3)
            .map(|k| {
                let p = &q[t[k].pos];
                [
                    (p[0] - (xmin + xmax) / 2.0) * s + w as f64 / 2.0,
                    h as f64 / 2.0 - (p[1] - (ymin + ymax) / 2.0) * s,
                    p[2],
                ]
            })
            .collect();
        let e1 = [q[t[1].pos][0] - q[t[0].pos][0], q[t[1].pos][1] - q[t[0].pos][1], q[t[1].pos][2] - q[t[0].pos][2]];
        let e2 = [q[t[2].pos][0] - q[t[0].pos][0], q[t[2].pos][1] - q[t[0].pos][1], q[t[2].pos][2] - q[t[0].pos][2]];
        let n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        let nl = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-30);
        let sh = ((n[0] * light[0] + n[1] * light[1] + n[2] * light[2]) / nl).abs();
        let col = [
            (70.0 * (0.35 + 0.65 * sh)) as u8,
            (110.0 * (0.35 + 0.65 * sh)) as u8,
            (170.0 * (0.35 + 0.65 * sh)) as u8,
        ];
        let (x0, x1) = (
            v.iter().map(|p| p[0]).fold(f64::MAX, f64::min).max(0.0) as usize,
            (v.iter().map(|p| p[0]).fold(f64::MIN, f64::max).min(w as f64 - 1.0)) as usize,
        );
        let (y0, y1) = (
            v.iter().map(|p| p[1]).fold(f64::MAX, f64::min).max(0.0) as usize,
            (v.iter().map(|p| p[1]).fold(f64::MIN, f64::max).min(h as f64 - 1.0)) as usize,
        );
        let d = (v[1][1] - v[2][1]) * (v[0][0] - v[2][0]) + (v[2][0] - v[1][0]) * (v[0][1] - v[2][1]);
        if d.abs() < 1e-12 {
            continue;
        }
        for yy in y0..=y1 {
            for xx in x0..=x1 {
                let (fx, fy) = (xx as f64, yy as f64);
                let l1 = ((v[1][1] - v[2][1]) * (fx - v[2][0]) + (v[2][0] - v[1][0]) * (fy - v[2][1])) / d;
                let l2 = ((v[2][1] - v[0][1]) * (fx - v[2][0]) + (v[0][0] - v[2][0]) * (fy - v[2][1])) / d;
                let l3 = 1.0 - l1 - l2;
                if l1 < 0.0 || l2 < 0.0 || l3 < 0.0 {
                    continue;
                }
                let z = l1 * v[0][2] + l2 * v[1][2] + l3 * v[2][2];
                let i = yy * w + xx;
                if z > zb[i] {
                    zb[i] = z;
                    img[i * 3..i * 3 + 3].copy_from_slice(&col);
                }
            }
        }
    }
    write_bmp(path, w, h, &img)
}

// ---------- STEP I/O ----------

fn load_step(path: &str) -> Result<Solid> {
    let bytes = std::fs::read(path)?;
    let table = Table::from_step_bytes(&bytes).map_err(|e| anyhow!("{e:?}"))?;
    let sh = table.shell.values().next().ok_or_else(|| anyhow!("kein CLOSED_SHELL"))?;
    let c = table.to_compressed_shell(sh).map_err(|e| anyhow!("{e:?}"))?;
    let healed = monstertruck_healing::extract_healed(c, 0.05).map_err(|e| anyhow!("heal: {e:?}"))?;
    let shell: Shell = healed.mapped(
        |p| *p,
        |c| Curve::try_from(c).expect("Kurve"),
        |s| Surface::try_from(s).expect("Fläche"),
    );
    let solid = Solid::try_new(vec![shell]).map_err(|e| anyhow!("try_new: {e:?}"))?;
    // Datei ist mm, intern Meter:
    Ok(builder::scaled(&solid, Point3::origin(), Vector3::new(1.0 / K, 1.0 / K, 1.0 / K)))
}

fn save_any(s: &Solid, path: &str) -> Result<()> {
    let big = builder::scaled(s, Point3::origin(), Vector3::new(K, K, K));
    if path.ends_with(".obj") {
        let m = big.triangulation(FINE * K).to_polygon();
        let f = std::fs::File::create(path)?;
        monstertruck_mesh::obj::write(&m, f).map_err(|e| anyhow!("{e:?}"))?;
        return Ok(());
    }
    let c = big.compress();
    let step = CompleteStepDisplay::new(StepModel::from(&c), Default::default());
    let txt = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| step.to_string()))
        .map_err(|_| anyhow!("STEP-Export panickte"))?;
    std::fs::write(path, txt)?;
    Ok(())
}

// ---------- Kommandos ----------

fn stats_line(name: &str, s: &Solid) -> String {
    let m = mesh_of(s);
    let (lo, hi) = bbox(&m);
    format!(
        "{name}: {} Faces, {:.0} mm³, bbox {:.0}×{:.0}×{:.0} mm",
        s.face_iter().count(),
        m.volume() * K * K * K,
        (hi.x - lo.x) * K,
        (hi.y - lo.y) * K,
        (hi.z - lo.z) * K
    )
}

impl App {
    fn get(&self, n: &str) -> Result<&Solid> {
        self.objs.get(n).ok_or_else(|| anyhow!("Objekt '{n}' unbekannt"))
    }

    fn boolean(&mut self, op: &str, out: &str, a: &str, b: &str, tol_mm: Option<f64>) -> Result<()> {
        let (sa, sb) = (self.get(a)?.clone(), self.get(b)?.clone());
        // Dispatcher-Light: disjunkte Boxen abfangen (Stufe 1)
        let (la, ha) = bbox(&mesh_of(&sa));
        let (lb, hb) = bbox(&mesh_of(&sb));
        let disjoint = ha.x < lb.x || hb.x < la.x || ha.y < lb.y || hb.y < la.y || ha.z < lb.z || hb.z < la.z;
        if disjoint {
            match op {
                "diff" => {
                    println!("  Boxen disjunkt: Ergebnis = {a} unverändert");
                    self.objs.insert(out.into(), sa);
                    return Ok(());
                }
                "and" => return Err(anyhow!("Boxen disjunkt: Schnittmenge leer, kein Objekt angelegt")),
                _ => println!("  Hinweis: Boxen disjunkt, Vereinigung wird zwei Komponenten"),
            }
        }
        let tols: Vec<f64> = match tol_mm {
            Some(t) => vec![t / K],
            None => SWEEP_MM.iter().map(|t| t / K).collect(),
        };
        let mut fails = 0usize;
        let mut last = String::from("-");
        for t in tols {
            let t0 = Instant::now();
            let r = match op {
                "and" => monstertruck_solid::and(&sa, &sb, t),
                "or" => monstertruck_solid::or(&sa, &sb, t),
                _ => monstertruck_solid::difference(&sa, &sb, t),
            };
            match r {
                Ok(s) if s.face_iter().count() > 0 => {
                    println!(
                        "  OK bei tol={} mm ({:.2} s, {} Fehlversuche)",
                        t * K,
                        t0.elapsed().as_secs_f64(),
                        fails
                    );
                    println!("  {}", stats_line(out, &s));
                    self.objs.insert(out.into(), s);
                    return Ok(());
                }
                Ok(_) => {
                    fails += 1;
                    last = "Ok, aber leeres Solid".into();
                }
                Err(e) => {
                    fails += 1;
                    last = format!("{e:?}");
                }
            }
        }
        Err(anyhow!("alle Toleranzen gescheitert, zuletzt: {last}"))
    }

    fn exec(&mut self, line: &str) -> Result<bool> {
        let t: Vec<&str> = line.split_whitespace().collect();
        if t.is_empty() || t[0].starts_with('#') {
            return Ok(true);
        }
        let f = |i: usize| -> Result<f64> {
            t.get(i)
                .and_then(|s| s.parse().ok())
                .ok_or_else(|| anyhow!("Zahl erwartet an Position {i}"))
        };
        match t[0] {
            "help" => println!(
                "box N sx sy sz | cyl N r h | frustum N r0 r1 h | uebergang N eckradius | load N datei.step\n\
                 move N dx dy dz | rot N ax ay az grad | scale N f | copy A B | del N | ls\n\
                 or|and|diff OUT A B [tol_mm] | vol N | check N\n\
                 save N datei.step|.obj | dump N axo|front|top datei.bmp [px]\n\
                 source datei | echo ... | exit    (Längen mm, Winkel Grad)"
            ),
            "echo" => println!("{}", t[1..].join(" ")),
            "exit" | "quit" => return Ok(false),
            "box" => {
                let s = boxsolid(f(2)? / K, f(3)? / K, f(4)? / K);
                println!("  {}", stats_line(t[1], &s));
                self.objs.insert(t[1].into(), s);
            }
            "cyl" => {
                let s = frustum(f(2)? / K, f(2)? / K, f(3)? / K)?;
                println!("  {}", stats_line(t[1], &s));
                self.objs.insert(t[1].into(), s);
            }
            "frustum" => {
                let s = frustum(f(2)? / K, f(3)? / K, f(4)? / K)?;
                println!("  {}", stats_line(t[1], &s));
                self.objs.insert(t[1].into(), s);
            }
            "uebergang" => {
                let s = uebergang(f(2)?)?;
                println!("  {}", stats_line(t[1], &s));
                self.objs.insert(t[1].into(), s);
            }
            "load" => {
                let s = load_step(t.get(2).ok_or_else(|| anyhow!("Pfad fehlt"))?)?;
                println!("  {}", stats_line(t[1], &s));
                self.objs.insert(t[1].into(), s);
            }
            "move" => {
                let s = builder::translated(self.get(t[1])?, Vector3::new(f(2)? / K, f(3)? / K, f(4)? / K));
                self.objs.insert(t[1].into(), s);
            }
            "rot" => {
                let ax = Vector3::new(f(2)?, f(3)?, f(4)?);
                let s = builder::rotated(self.get(t[1])?, Point3::origin(), ax.normalize(), Rad(f(5)?.to_radians()));
                self.objs.insert(t[1].into(), s);
            }
            "scale" => {
                let k = f(2)?;
                let s = builder::scaled(self.get(t[1])?, Point3::origin(), Vector3::new(k, k, k));
                self.objs.insert(t[1].into(), s);
            }
            "copy" => {
                let s = self.get(t[1])?.clone();
                self.objs.insert(t[2].into(), s);
            }
            "del" => {
                self.objs.remove(t[1]);
            }
            "ls" => {
                for (n, s) in &self.objs {
                    println!("  {}", stats_line(n, s));
                }
            }
            "or" | "and" | "diff" => {
                let tol = t.get(4).and_then(|s| s.parse().ok());
                self.boolean(t[0], t[1], t[2], t[3], tol)?;
            }
            "vol" => {
                let m = mesh_of(self.get(t[1])?);
                println!("  {:.1} mm³", m.volume() * K * K * K);
            }
            "check" => {
                let s = self.get(t[1])?;
                let m = mesh_of(s);
                let naked = naked_edges(&m);
                println!("  {}", stats_line(t[1], s));
                println!(
                    "  Dreiecke {}, Randkanten {} -> {}",
                    m.faces().triangle_iter().count(),
                    naked.len(),
                    if naked.is_empty() { "dicht" } else { "UNDICHT" }
                );
            }
            "save" => {
                save_any(self.get(t[1])?, t[2])?;
                println!("  geschrieben: {}", t[2]);
            }
            "dump" => {
                let px: usize = t.get(4).and_then(|s| s.parse().ok()).unwrap_or(900);
                dump(&mesh_of(self.get(t[1])?), t[2], px, t[3])?;
                println!("  Bild: {}", t[3]);
            }
            "source" => {
                let txt = std::fs::read_to_string(t[1])?;
                for l in txt.lines() {
                    println!("» {l}");
                    if !self.exec(l)? {
                        return Ok(false);
                    }
                }
            }
            other => return Err(anyhow!("Kommando '{other}' unbekannt (help)")),
        }
        Ok(true)
    }
}

fn main() {
    let mut app = App { objs: BTreeMap::new() };
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() {
        let stdin = std::io::stdin();
        for line in stdin.lock().lines().map_while(Result::ok) {
            println!("» {line}");
            match app.exec(&line) {
                Ok(true) => {}
                Ok(false) => break,
                Err(e) => println!("  FEHLER: {e}"),
            }
        }
    } else {
        for a in &args {
            if let Err(e) = app.exec(&format!("source {a}")) {
                println!("  FEHLER: {e}");
            }
        }
    }
}
