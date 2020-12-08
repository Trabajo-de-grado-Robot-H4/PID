# DESCRIPCIÓN GENERAL
En este repositorio se encuentran los archivos para el control del robot paralelo H4, a continuación se describe la finalidad de cada uno.

  lecturaEncoder.py
  ->  Se encarga de verificar el correcto funcionamiento del enconder de un motor, en su ejecución se muestra el valor de la lectura del encoder mediante ventana de comandos.
    
    
  movimientoMotor.py
  ->  Verifica el correcto funcionamiento del motor a través del puente H, ejectua un movimiento horario de 10s acelerado progresivamente, una pausa de 1s y un movimiento
    antihorario de 10s acelerado progresivamente.
  control_pid_motor.py 
  -> permite controlar la posicion en grados de un motor dc con la ayuda de un control PID usando el enconder incremental como sensor de posicion y el puente H para enviar una señar pwm al motor.
  
