# Prova in laboratorio sul robot

Procedura per il primo collaudo fisico. Non è ancora stata eseguita: finora solo
test di logica, test ROS del blocco velocità e un preflight da fermo.

## Prima di accendere

Il robot a terra, nell'area libera concordata, con un operatore accanto che
conosca l'arresto fisico indicato dal tutor. L'arresto software passa dal computer
del robot e non sostituisce quello fisico.

Controllare che nessun teleop o altro processo pubblichi su `/cmd_vel`:

    ros2 topic info /cmd_vel -v

Deve comparire solo `motion_guard`. Se ce ne sono altri, la corsa non parte finché
non vengono chiusi: due publisher sullo stesso topic si sovrascrivono a vicenda e
il robot può restare fermo senza che nulla segnali un errore.

## Avvio

Dal terminale SSH sul robot:

    cd /home/students/ros_ws/src/asr_summer_school_challenge
    bash tools/robot_mission.sh launch 60.0

Avvia sensori, SLAM e Nav2, ma non muove nulla: il gate parte disarmato. Da un
secondo terminale:

    bash tools/robot_mission.sh status
    bash tools/robot_mission.sh arm
    bash tools/robot_mission.sh start

`stop` revoca il permesso di movimento in qualsiasi momento.

Il terzo argomento di `launch` è `max_radius` e vale 0, cioè nessun limite di
posizione: la missione è limitata dal tempo e dalla riserva per il rientro. Per un
primo collaudo prudente in poco spazio si può passare un valore in metri, per
esempio `bash tools/robot_mission.sh launch 60.0 1.5`, che riporta il robot verso
la partenza appena supera quella distanza.

## Se qualcosa non si muove

`/safety/status` dice quale condizione sta bloccando: sensore assente, heartbeat
scaduto, comando scaduto, ostacolo entro 20 cm, o gate disarmato. È la prima cosa
da leggere, prima di toccare qualsiasi parametro.

Per una registrazione in sola lettura di velocità, laser, odometria, immagini e
detection:

    bash tools/robot_mission.sh diagnose 30

Segnala anche publisher multipli su `/cmd_vel` e nomi di nodo duplicati, che sono
il sintomo tipico di un lancio precedente rimasto aperto. Non termina niente.

## Due trappole che fanno perdere tempo

Sono state entrambe osservate sul robot 08. In tutti e due i casi il sintomo è
identico: nessun errore da nessuna parte, e il robot fermo.

### Il bringup dei tutor ha bisogno del router Zenoh

Con `RMW_IMPLEMENTATION=rmw_zenoh_cpp` la scoperta fra processi passa da un router
su `localhost:7447`. Senza router ogni processo resta in una sessione isolata: il
teleop pubblica su `/cmd_vel` e `turtlebot3_node` non lo sente mai. `ros2 node
list` non mostra nulla e `ros2 topic list` solo `/parameter_events` e `/rosout`,
anche se il bringup nel suo terminale scrive `Run!` — quel messaggio dice solo che
la seriale verso l'OpenCR si è aperta.

Il router va avviato prima, in un terminale che resta aperto:

    ros2 run rmw_zenoh_cpp rmw_zenohd

Per capire in un secondo se è il problema: `ss -ltn | grep 7447`. Se la porta è
chiusa, non c'è router. Vale anche per una registrazione con `ros2 bag record`,
che altrimenti registra il vuoto.

### Il nostro stack e quello dei tutor non convivono

`autonomy.launch.py sensors:=true` include `turtlebot3_bringup/robot.launch.py`,
quindi avvia da sé i driver, la camera, il rilevatore AprilTag e SLAM. È
autosufficiente e va lanciato **con il bringup dei tutor spento**: altrimenti due
`turtlebot3_node` si contendono `/dev/ttyACM0`.

I nostri script esportano `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, mentre l'ambiente
consigliato sul robot usa Zenoh. Dentro il nostro lancio è coerente e funziona,
ma da un altro terminale SSH non si vede niente della missione finché non si
esporta lo stesso middleware:

    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=8 ROS_LOCALHOST_ONLY=1

Prima di gridare al guasto, controllare questo. La combinazione opposta — il
nostro stack con `sensors:=false` appoggiato al bringup dei tutor — non è mai
stata provata e con i middleware diversi non può funzionare: Nav2 non vedrebbe
`/scan`.

## Correzioni già applicate nel nostro package

I submodule dei tutor restano invariati; quanto segue è nel nostro pacchetto.

- DWB entrava in rotazione finale a 25 cm mentre il goal checker accettava 20 cm.
  Ora usano entrambi 20 cm. Era una configurazione capace di bloccare
  l'avanzamento vicino ai goal.
- Il detector AprilTag nel namespace `/camera` aveva remap relativi che
  risolvevano in `/camera/camera/color/...`, mentre la RealSense pubblica su
  `/camera/color/...`. Ora i remap sono assoluti.
- La dimensione tag configurata è 0,16 m, ereditata dalle configurazioni del
  corso. Misurare il bordo codificato di un tag reale prima di giudicare la
  precisione delle posizioni.

## Cosa resta da verificare sul robot

Middleware Zenoh e dominio ROS, modello di camera reale, deriva odometrica della
posa di partenza memorizzata, e il confronto RPP contro DWB con numeri alla mano
prima di cambiare la configurazione del robot.
