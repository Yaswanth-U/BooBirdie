# BooBirdie
All the necessary code and resources to build your own RC Ornithopter.

## Description
BooBirdie is a remote controlled RC (An ornithopter (from Ancient Greek ὄρνις (órnis) 'bird' and πτερόν (pterón) 'wing') is an aircraft that flies by flapping its wings.[Source: Wikipedia])

the aircraft uses two N20 motors with encoders for its propulsion one for each wing and a servo for the tail to control pitch.

### Control System
the ornithopter works on the principle of differential thrust(control method using unequal power between engines on a multi-engine vehicle to create yaw (turning) moments) the two motors in the wings go out of phase to achieve this but for the aircraft to go straight after turning both the motors should go in phase to achieve this Flight controller needs precise control of speed as well as angle of each motor making encoders a crusial part of the control system. The tail is attached to a microservo which controls the pitch.

### Power Management
| Component    | Current Drawn |
| ------------ | ------------- |
| Motors       | 60mA*2        |
| Servo        | 200mA         |
| Raspberry pi | 90mA          |
| Radio TX     | 100mA         |
| **Total**    | 510mA         |


## Flapping wing lift thrust calculator
html based calculator to fing thee thrust and lift of a given ornithopter based on the given configuration(Made based on research paper: [An Aerodynamics Calculation Method of a Flapping Wing
Flying Robot Based on State-Space Airloads Theory*
Hui Xu, Erzhen Pan, Dong Xue, Wenfu Xu*, Senior, IEEE, Yuanpeng Wang, Xu Liang](https://doi.org/10.1109/ROBIO49542.2019.8961808:))
### Inputs

1. Weight W (g)
2. Wingspan b (m)
3. Mean chord c (m)
4. Planform efficiency η
5. Flapping frequency f (Hz)
6. Flapping amplitude Φ (°)
7. Forward speed U (m/s)
8. Angle of attack α (°)
9. Parasitic drag coeff C_D0
### Output(Results)
1. Mean lift L̄
2. Lift / Weight
3. Mean thrust T̄
4. lift & thrust vs flapping frequency Graph