#pragma config(Motor,  port2,           r_drive,       tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port3,           l_drive,       tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port4,           lift,          tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port5,           l_claw,        tmotorServoStandard, openLoop)
#pragma config(Motor,  port6,           r_claw,        tmotorServoStandard, openLoop, reversed)
#pragma config(Motor,  port7,           grabber,       tmotorServoStandard, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//Make sure to use Natural Language 2.0

bool reversed = false;
bool halfway = true;
bool closed = true;

void drive() {
	int deadzone = 25;
	if(reversed) {
		if(abs(vexRT[Ch3]) > deadzone) {
			motor[l_drive] = -vexRT[Ch3];
		}
		else {
			motor[l_drive] = 0;
		}
		if(abs(vexRT[Ch2]) > deadzone) {
			motor[r_drive] = -vexRT[Ch2];
		}
		else {
			motor[r_drive] = 0;
		}
	}
	else {
		if(abs(vexRT[Ch2]) > deadzone) {
			motor[l_drive] = vexRT[Ch2];
		}
		else {
			motor[l_drive] = 0;
		}
		if(abs(vexRT[Ch3]) > deadzone) {
			motor[r_drive] = vexRT[Ch3];
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

void light_grabber() {
	if(vexRT[Btn7D] == 1){
		halfway = true;
		motor[grabber] = 87;
	}
	if(vexRT[Btn7U] == 1){
		if(halfway) {
			motor[grabber] = 60;
			halfway = false;
			while(vexRT[Btn7U] == 1) {}
		}
		else {
			motor[grabber] = 10;
		}
	}
}

void claw() {
	if(vexRT[Btn8U] == 1){
		if(closed) {
			motor[l_claw] = -127;
			motor[r_claw] = -127;
			closed = false;
		}
		else {
			motor[l_claw] = 0;
			motor[r_claw] = 0;
			closed = true;
		}
		while(vexRT[Btn8U] == 1) {}
	}

	if(vexRT[Btn5U] == 1){
		motor[lift] = 127;
	}
	else if(vexRT[Btn5D] == 1){
		motor[lift] = -80;
	}
	else {
		motor[lift] = 0;
	}
}

void auto(){
	int toward = 170;
	int back = 275;
	if(vexRT[Btn6D] == 1){
		motor[grabber] = 87;
		halfway = true;

		float time = 0.5;
		int from = 20;
		int to = 100;
		for(int i = from; i < to; i++) {
			motor[l_drive] = -((i + 80) - abs(i - 80)) / 2;
			motor[r_drive] = -i;
			wait(time / (to - from));
		}

		int i = 0;
		while(i != toward){
			if(vexRT[Btn6U] == 1){
				return;
			}
			wait(0.01);
			i++;
		}
		motor[l_drive] = 0;
		motor[r_drive] = 0;
		motor[l_claw] = 0;
		motor[r_claw] = 0;
		closed = true;
		wait(0.5);
		motor[l_drive] = 80;
		motor[r_drive] = 80;
		i = 0;
		while(i != back){
			if(vexRT[Btn6U] == 1){
				return;
			}
			wait(0.01);
			i++;
		}
	}
}

void adjust() {
	if(vexRT[Btn8L] == 1) {
		motor[l_claw] = -127;
		motor[r_claw] = -127;
		wait(0.3);
		motor[l_drive] = 65;
		motor[r_drive] = 65;
		wait(0.25);
		motor[l_drive] = 0;
		motor[r_drive] = 0;
		wait(0.3);
		motor[l_claw] = 0;
		motor[r_claw] = 0;
		closed = true;
	}
}
task main()
{
	motor[grabber] = 87;
	motor[l_claw] = 0;
	motor[r_claw] = 0;
	while(true) {
		drive();
		light_grabber();
		claw();
		auto();
		adjust();
	}
}
