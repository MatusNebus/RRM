# block1_nebus

Balík pre učenie bodov, prehrávanie trajektórie a manuálne riadenie robota.

## Model robota (`robot_name`)

Model robota sa vyberá argumentom `robot_name` pri spustení launch súboru.

Podporované hodnoty:

- `advancedArm` (predvolené)
- `simpleArm` (zatiaľ placeholder, kým nedoplníš vlastný URDF)

URDF súbory sú v `robot/rrm_simple_robot_model/urdf`:

- `advancedArm.urdf`
- `simpleArm.urdf`

## Stručný popis súborov v `src/`

- `src/control_node.cpp`: Interaktívne menu, volá služby pre move, save, play a clear. (funguje to len pre simpleArm)
- `src/logger_node.cpp`: Vypisuje/loguje stav kĺbov z topicu `joint_states`.
- `src/play_trajectory_client_node.cpp`: Klient, ktorý volá službu `/play_trajectory`.
- `src/play_trajectory_server_node.cpp`: Načíta `teach_points.txt` a postupne volá `move_command`.
- `src/teach_point_client_node.cpp`: Klient, ktorý uloží aktuálnu polohu cez `/save_point`.
- `src/teach_point_server_node.cpp`: Server pre `/save_point` a `/clear_points`, zapisuje do `teach_points.txt`.
- `src/teleop_node.cpp`: Jednoduché manuálne ovládanie cez `move_command`.

## Rýchly štart (odporúčaný postup)

### 1. Build workspace

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

### 2. Spusti celý systém (Terminál A)

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch block1_nebus block1_system.launch.xml robot_name:=advancedArm
```

Pre `simpleArm` použi:

```bash
ros2 launch block1_nebus block1_system.launch.xml robot_name:=simpleArm
```

### 3. Spusti ovládanie (Terminál B)

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run block1_nebus control_node
```

### 4. Voliteľná kontrola služieb (Terminál C)

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 service list | grep -E "save_point|clear_points|play_trajectory|move_command"
```

Mal by si vidieť minimálne:

- `/move_command`
- `/save_point`
- `/clear_points`
- `/play_trajectory`

## Samostatné spúšťanie nodov

- `control_node` samostatne: Nie, potrebuje bežať servery (`teach_point_server_node`, `play_trajectory_server_node`) a simulátor (`rrm_sim`).
- `teach_point_server_node` samostatne: Áno, ale potrebuje bežať zdroj `joint_states` (simulátor).
- `play_trajectory_server_node` samostatne: Áno, ale potrebuje bežať službu `move_command` (simulátor) a súbor `teach_points.txt`.
- `teach_point_client_node` samostatne: Áno, ale potrebuje `/save_point` server.
- `play_trajectory_client_node` samostatne: Áno, ale potrebuje `/play_trajectory` server.
- `teleop_node` samostatne: Áno, ale potrebuje `/move_command` server.
- `logger_node` samostatne: Áno, ale potrebuje topic `joint_states`.

## Služby

- `/move_command`: Pohyb robota (server je v `rrm_sim`).
- `/save_point`: Uloženie aktuálnej polohy a rýchlosti.
- `/clear_points`: Vymazanie `teach_points.txt`.
- `/play_trajectory`: Prehratie uložených bodov.

## Typické problémy

- `Service /save_point not available`: Beží len `control_node`, ale nebeží `teach_point_server_node`.
- `incorrect size of desired positions`: Nesúlad počtu kĺbov medzi klientom a simulátorom; systém je upravený tak, aby podporoval aj 3-prvkový príkaz pre `advancedArm`.

## Poznámka k `simpleArm`

`simpleArm.urdf` je momentálne placeholder. Kým ho nedoplníš validným obsahom URDF, tento variant nemusí byť použiteľný na vizualizáciu ani simuláciu.
