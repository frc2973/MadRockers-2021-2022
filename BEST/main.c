#pragma config(Motor,  port2,           r_drive,       tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port3,           l_drive,       tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port6,           servo,         tmotorServoStandard, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

bool reversed = false;

void drive() {
	int deadzone = 25;
	if(reversed) {
		if(abs(vexRT[Ch2]) > deadzone) {
			motor[l_drive] = -vexRT[Ch2];
		}
		else {
			motor[l_drive] = 0;
		}
		if(abs(vexRT[Ch3]) > deadzone) {
			motor[r_drive] = -vexRT[Ch3];
		}
		else {
			motor[r_drive] = 0;
		}
	}
	else {
		if(abs(vexRT[Ch3]) > deadzone) {
			motor[l_drive] = vexRT[Ch3];
		}
		else {
			motor[l_drive] = 0;
		}
		if(abs(vexRT[Ch2]) > deadzone) {
			motor[r_drive] = vexRT[Ch2];
		}
		else {
			motor[r_drive] = 0;
		}
	}

	if(vexRT[Btn8R] == 1) {
		while(vexRT[Btn8R] == 1) {}
		reversed = !reversed;
	}
}

void move_servo() {
	if(vexRT[Btn7U] == 1){
		while(vexRT[Btn7U] == 1) {}
		motor[servo] = 127;
	}
	if(vexRT[Btn7D] == 1){
		while(vexRT[Btn7D] == 1) {}
		motor[servo] = -127;
	}
}

task main()
{
	while(true) {
		drive();
		move_servo();
	}
}
