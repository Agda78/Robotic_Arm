# Robotic Arm
Codice implementato per la board Nucleo-F401RE, con una expansion board bluetooth ed uno specifico Sensor Tile

La struttura aggiunta nel codice rispetto alla libreria di comunicazione:
- **servo.h**: Libreria di basso livello per l'implementazione del controllo dei motori mediante lo specifico pwm
- **arm_control.h**: Libreria di "medio" livello per l'implementazione del controllo effettivo a partire dai dati provenienti dall'accelerometro
- **it_callbacks.c**: Implementazione delle Callback per il servizio delle ISR, sia quelle dedicate alla sicurezza (associate ai pin GPIO), sia quella periodica per l'applicazione del controllo (quella associata al TIM 4)
