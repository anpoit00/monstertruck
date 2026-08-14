# ÜBERGABE — monstertruck Boolean-Untersuchung

Stand: 2026-08. Basis: virtualritz/monstertruck, Commit 17c0b1d5 (Verhalten identisch
zu 609c1b5 verifiziert). Toolchain: Rust 1.91.1. Arbeitszweig: `arbeit`.
Dieses Dokument fasst eine mehrwöchige Messkampagne zusammen; alle Zahlen sind
reproduzierbar über die Regressionsskripte (unten).

## Inhalt dieses Repos gegenüber upstream

- `monstertruck-solid/examples/draw.rs` — DRAW-artige Kommando-Shell (512 Zeilen).
  Kommandos: box, cyl, frustum, uebergang (gerundetes Übergangsstück, Eckradius
  als Parameter), load (STEP mit Healing), move/rot/scale/copy/del/ls,
  or/and/diff (mit automatischem Toleranz-Sweep 0.05–8 mm und
  Disjunkt-Dispatcher), vol, check (Volumen + verschweisster Randkantentest
  → dicht/undicht), save (STEP/OBJ), dump (BMP-Software-Render axo/front/top,
  kein GPU), source, help. Alle Längen mm, intern Meter.
- `session1.dr`, `session2.dr` — Regressionsskripte mit festen Sollwerten.
- `monstertruck-solid/Cargo.toml` — zwei zusätzliche Dependencies
  (monstertruck-healing, monstertruck-mesh, per path).

## Reproduktion

    cargo build --release --example draw -p monstertruck-solid
    ./target/release/examples/draw session1.dr
    ./target/release/examples/draw session2.dr

Sollwerte session1: Import 9 978 975 mm³/10 F/dicht · diff Ø60: tol=1 mm,
14 F, 9 458 511, dicht · Hosenrohr-or (2 schiefe Kegelstümpfe r80/r139.5,
±24.235°): tol=0.05, 12 F, 25 645 556, dicht · disjunkte diff → "Ergebnis = a
unverändert".
Sollwerte session2: uebergang(1 mm) 9 969 254/10 F/dicht · diff Ø200 z=130:
tol=2, 16 F, Rest 4 541 307, dicht · diff Ø60: tol=1, 14 F, 9 475 886, dicht.

## Kernbefunde (alle gemessen, nicht vermutet)

1. **Skalenabhängigkeit.** Toleranzen/Epsilons sind absolut. In mm scheitern
   Booleans, die in Metern laufen. Konvention: intern Meter, Ausgabe mm.
   Erste funktionierende Toleranz ≈ Bauteilgrösse/300, feinere scheitern oft.
2. **`Ok` garantiert nichts.** Beobachtet: Ok mit leerem Solid (Werkzeug
   disjunkt bei difference), Ok mit Phantomgeometrie (senkrechte Bohrung:
   Werkzeug über volle Länge abgezogen, 288 Vertices unterhalb der
   Bodenfläche, von try_new als Manifold akzeptiert). Pflicht nach jedem
   Boolean: Faces>0, Volumenbilanz, Hüllentest. `check` in draw macht das.
3. **Degenerierte Kegelflächen (kollabierte Kante / Spitze).** Jede boolesche
   Beteiligung → EmptyOutputShell, unabhängig davon, wo geschnitten wird
   (Fläche wird als Ganzes parametrisiert; ∂S/∂u=0 an der Spitze). Schwelle
   exakt vermessen: Schnittkreis auf z=200, Grenze zur Eckfläche 29.98 mm —
   r=30 läuft, r=34 tot. Abhilfe konstruktiv: Grundecken ≥1 mm ausrunden
   (Kommando `uebergang`); danach laufen auch Schnitte durch die Eckflächen
   (16 Faces, ±0.08 % gegen Netz-Referenz).
4. **Koinzidenz / Same-Domain: nicht unterstützt.** Deckungsgleiche Ebenen →
   NotClosedShell bei and/or/difference. Todeszone ±≈1 µm, **toleranz-
   unabhängig** (tol 0.0005–0.008 identisch; es zählt n0×n1≈0, kein Abstand).
   Untermass 1 µm "läuft" und lässt Mikro-Deckel stehen. Kipp-Schwelle
   zwischen 1e-5 und 1e-4 rad. **Gedrehte Lage (geometrisch koinzident,
   numerisch nicht bitgleich): Hänger >780 s ohne Rückkehr** statt Fehler —
   Schrittbudget zwingend. Nur-Kante-Kontakt → NotClosedShell; Nur-Ecke →
   Ok+leer. Koaxiale Zylinder: 1.5 mm Radialluft scheitert, 2 mm läuft.
   Enthaltensein (Werkzeug ganz innen ODER umschliessend) → NotClosedShell.
   Ausnahme, die funktioniert: Versatz in beiden Richtungen
   (adjacent_cubes_or, Test existiert upstream und ist grün).
   **plane_cut/clip_half_space_z beherrschen bündige Ebenen korrekt** (4/4
   Proben, auch exakt auf Deck-/Bodenfläche) — für bündige Schnitte diese
   Route, nie difference mit Quader.
5. **Transversale Fälle sind gut.** Zwei schiefe Kegelstümpfe (Hosenrohr):
   and/or/diff korrekt, Monte-Carlo 8M Punkte, Abweichung ≤0.4 %, Naht
   z=130.1 exakt. Zylinder-Variante: −0.05 %. Toleranz dort gutmütig über
   Faktor 640. NICHT konstruieren: beide Schenkel mit gemeinsamer Deckfläche
   (→ Koinzidenz, alles scheitert).

## Lokalisierte Bruchstellen im Code

- `monstertruck-meshing/src/analyzers/collision.rs`, collide_seg_triangle:
  koplanar → 0/0 → NaN → keine Saatsegmente für den Marcher.
- `monstertruck-geometry` IntersectionCurve::search_nearest_point: 4. Gleichung
  n·diff=0 mit n=n0×n1; parallel → 4. Jacobi-Zeile null → Newton singulär.
  (Mathematisch prinzipiell: Lösungsmenge ist dort Fläche, keine Kurve.)
- `transversal/classic/loops_store.rs`: bei Koinzidenz werden Kanten der
  Gegen-Shell als Schnittkurven eingesammelt → degenerierte 2-Kanten-Loops
  (Befund tsukimizake).
- `transversal/classic/divide_face.rs`, divide_one_face: Face::debug_new ==
  new_unchecked im **Release** → nicht-einfache Wires werden stillschweigend
  Flächen (Befund gabrielgrant; wahrscheinliche Ursache des Phantomrohrs).
- ShapesOpStatus { Unknown, And, Or } — kein Koinzidenzzustand.
  Wichtig: difference ist intern and(A, ¬B) (Fehlermeldungen tragen op "and").

## Externe Arbeiten (geklont und gelesen)

- **gabrielgrant/monstertruck**, Branch `tr/fix/coincident-plane-booleans`:
  NaN-Guard (normalisiert mit nor.magnitude() → echte Ebenendistanz,
  massstabsrichtig) + try_new statt debug_new. Klein, sauber begründet,
  direkt portierbar. Schliesst die Lücke nicht, macht sie ehrlich.
- **tsukimizake/truck**, `fix-coplanar-boolean-operations-2`: 2 Monate,
  aufgegeben; gelöschte report.md in Historie. Sein gelöster Fall
  (adjacent_cubes) ist in monstertruck bereits anders gelöst.
- **SolveSpace** (Container-verifiziert, Testsuite 262/926 grün inkl.
  boolean_coplanar_union, knife_edge, 4×tangent): 4-Zustands-Klassifikation
  SURF_INSIDE/OUTSIDE/COINC_SAME/COINC_OPP per Strahlwurf; Entscheidungstabelle
  KeepRegion (boolean.cpp ~265ff): UNION keepB=outside||coincSame,
  DIFFERENCE keepA=outside||coincOpp, INTERSECTION keepB=inShell||coincSame.
  Toleranz DOTP_TOL=1e-5 **dimensionslos** (Richtungskosinus). Exakte Zweige
  Ebene/Ebene, Ebene/Extrusion vor numerischem Marching. ABER: zwei schiefe
  **Kegel** dort undicht (leaks=1) bei booleanFailed=false — Marching-Pfad;
  Zylinder (exakter Zweig) korrekt. Ab 1 mm Konizität auf 373 mm kippt es.
  API-Falle: Profilnormale muss invertiert werden (sbls.normal.ScaledBy(-1)),
  sonst Trimms invertiert → alles SURF_OUTSIDE, stilles Falschergebnis.
- **GoTools/SISL** (SINTEF): Identity::identicalSfs → 0/1/2/3 (nicht/koinzident/
  1in2/2in1); Prüfung Randkurven paarweise + 10 innere Isokurven via
  closestPoint, 10 % Randeinrückung. Koinzidenz wird VOR dem transversalen
  Pfad geprüft (compute-Kaskade), microCase() für zu-klein-zum-Teilen.
  SISL: Normalenkegel (s1990) als Separationskriterium — kein Saatpunkt nötig.
  GoTools hat KEINE Solid-Booleans (booleanIntersect(SurfaceModel) auskommentiert).
- Netz-Boolesche mit gelöster Koinzidenz (als Prüforakel geeignet): CGAL Nef,
  Manifold, libigl/Blender exact. 2D-Boolean in Rust: i_overlay,
  cavalier_contours (Bögen!).

## Plan (drei Stufen; 0→1 empfohlen, 2 nur bei echtem Bedarf)

**Stufe 0 — Ehrlichkeit (Stunden).** NaN-Guard collide_seg_triangle;
try_new statt debug_new in divide_one_face; Schrittbudget im Marcher
(Hänger → Err); difference bei disjunkten BBoxen → A zurück, leeres Ok →
Fehler. Upstream-taugliche PRs.

**Stufe 1 — Erkennen & Ausweichen (Tage), ausserhalb des Kerns.**
classify_pair je Flächenpaar: (a) analytisch via TryIntoAnalyticSurfaceKind
(Ebene/Ebene: Richtungskosinus dimensionslos + Aufpunktabstand;
Zylinder/Zylinder: Achsen/Radien), (b) sonst GoTools-Port: Randkurven +
10 Isokurven via SearchNearestParameter, (c) Normalenkegel als
Transversalitäts-Garantie. Dispatcher: disjunkt/eingebettet direkt
beantworten (Hohlraum = Solid mit 2 Shells — verifizieren!); koinzident →
plane_cut-Route ODER ε-Störung (Ebenen 2–5 µm, immer ÜBERSTAND, nie
Untermass; gekrümmt: ablehnen — 2 mm Radialluft ist Massänderung) ODER
Fehler mit Diagnose. Längentoleranzen relativ zur BBox-Diagonale.

**Stufe 2 — Kernumbau (Wochen).** CoincidentRegion je Flächenpaar; gemeinsame
Region als **2D-Boolean im uv** der geteilten Fläche (i_overlay/
cavalier_contours); Regionsgrenzen als getaggte Trimmkurven (ersetzt die
degenerierten 2-Kanten-Loops konstruktiv); KeepRegion-Tabelle nach
SolveSpace — dank difference=and(A,¬B) genügen and+or (Same↔Opp unter
Inversion). Härtester Rest: gemischter Fall (teils koinzident, teils
transversal; GoTools "PAC"): Region ausschneiden, Rest marchen, an
Regionsgrenze nähen.
Sackgassen (belegt): Newton koinzidenzfähig machen (singulär aus Prinzip);
Toleranz erhöhen (Zone toleranzunabhängig); Punkt-Stichproben; Fuzzy ohne
Zustandserweiterung.

## Nächste konkrete Schritte

1. Stufe-0-Patches auf `arbeit`; nach jedem Patch beide Skripte (Sollwerte oben).
2. Prüforakel härten: Manifold oder CGAL als exakte Volumen-/Topologiereferenz
   statt Monte-Carlo (~20 Zeilen Anbindung).
3. draw ausbauen: `serve` (WebSocket + three.js-Viewer, Prototyp existiert als
   statisches HTML mit Randkanten-Overlay), Warnung "kollabierte Kanten" in
   `check` (die 4 Nulllängen-Kanten der Original-Eckflächen werden derzeit
   gefiltert — sie sind der Singularitäts-Indikator).
