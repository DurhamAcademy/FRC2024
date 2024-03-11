// package frc.robot.commands;
//
// import com.revrobotics.CANSparkLowLevel;
// import com.revrobotics.CANSparkMax;
//
//
// public class IntakeCommand {
//    //private static double DEADBAND = 0.1;
//    private CANSparkMax motor;
//     CANSparkMax extend = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushed);
//     CANSparkMax spin = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
//     public void spin(){
//
//
//         CANSparkMax motor = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
//         motor.set(0.5); //maybe not 1.0
//         while(true) { //just spin indefinitely
//             try{
//                 Thread.sleep(100);// pause for a bit
//
//             }catch (InterruptedException e){
//                 e.printStackTrace();
//             }
//
//
//         }
//
//
//     }
//     public void extend() {
//         CANSparkMax motor = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushed);
//         //motor id and what type of motor it is
//         extendMotor(motor);
//         while(true) {
//             try{
//                 Thread.sleep(100);
//             }catch(InterruptedException e){
//                 e.printStackTrace();
//
//             }
//         }
//
//     }
//     private static void extendMotor(CANSparkMax motor){
//         double motorSpeed = 0.5;
//         motor.set(motorSpeed);
//     }
//
//
//
//     public void periodic(){ //motor constant (extend, spin)
//         //idk how to periodic
//    }
// }
