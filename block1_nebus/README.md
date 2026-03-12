# block1_nebus

Balík pre riadenie robotického ramena. Umožňuje učenie polohy, prehrávanie trajektórii a diaľkové ovládanie.

## Hlavné komponenty

### Teach Point Server
Server pre ukladanie polohy robota do súboru. Pri spustení servera sa v adresári balíka vytvorí súbor `teach_points.txt`, kde sa ukladajú jednotlivé body.

### Teach Point Client
Interaktívny klient pre komunikáciu s teach point serverom. Umožňuje zadať rýchlosť a uložiť aktuálnu polohu robota.

### Play Trajectory Server
Server pre prehrávanie uložených bodov z `teach_points.txt`. Načíta body zo súboru a pošle ich na riadenie robota.

### Play Trajectory Client
Interaktívny klient pre komunikáciu s play trajectory serverom.

### Control Node
Centrálny riadiaci uzol. Komunikuje so všetkými ostatnými servermi a koordinuje operácie (učenie, prehrávanie, vymazávanie bodov).

### Teleop Node
Uzol pre diaľkové ovládanie. Umožňuje manuálne riadenie robota v reálnom čase.

### Logger Node
Zaznamenáva stav kĺbov robota.

## Súbory

- `teach_points.txt` - Ukladá body učenia. Vytvorí sa automaticky v adresári balíka. Formát: `ID poloha1 poloha2 poloha3 rýchlosť`
- `launch/block1_system.launch.xml` - Spúšťa všetky uzly systému

## Služby

- `/save_point` - Uloží aktuálnu polohu s danou rýchlosťou
- `/play_trajectory` - Prehráva uložené body
- `/clear_points` - Vymaže všetky uložené body

## Spustenie

Zapni všetky uzly:
```bash
ros2 launch block1_nebus block1_system.launch.xml
```
Potom v druhom termináli spusti controll node:
```bash
ros2 run block1_nebus controll_node
```
