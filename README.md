# SinCos-Encoder-Interface
STM32 Hiperface SinCos Encoder circuit and software
These document is based on HIPERFACE Specification for motor control feedback, This PDF is in the Root folder.
## The encoder have 2 differential outputs (+-sin and +-cos) as you can see in this Figure by Sick 

![image](https://user-images.githubusercontent.com/84080967/122963507-b5792700-d35c-11eb-9521-77834e66387c.png)

## The analog circuit:
![image](https://user-images.githubusercontent.com/84080967/122963321-895da600-d35c-11eb-82b1-97a9dc8a3f3e.png)

```
//ENCODER VARIABLES
int indexx;
long int interp;
long int XY;
long int t;
int Sin,Cos,X,Y,A;
uint16_t B,B2,B3;
long int tits;
int octant,octant2;

extern float Error_d,SP_d,Pterm_d,Pterm2_d,Iterm_d,Iterm2_d;
extern float Error_q,SP_q,Pterm_q,Pterm2_q,Iterm_q,Iterm2_q;
extern float thetaSum,theta_eSum;
long int thetaI,Turns,Theta_Turns;
float theta,theta2,theta_e,AbsTheta,AbsTheta2;
int Sa,Ca,S,C;

int ATAN_Table[33] = 
{
58,331,653,976,1298,1617,1934,2247,2555,2860,3159,3453,3742,4025,4301,4572,
4836,5093,5344,5588,5826,6057,6282,6500,6712,6917,7116,7310,7497,7679,7855,8026,8192
};

void Atan_interp(void) 
{
	indexx = A/1024;//(A>>10 );
	A = (A - (indexx<<10));
	interp = ATAN_Table[indexx+1] - ATAN_Table[indexx];
	interp = interp*A;
	A = interp/1024;
	A = A + ATAN_Table[indexx];
}
void Enc_Process(void)
{
//Octant and Fine angle calculation
if (Sin >= 0) 
{
	S = 1;
	if (Cos >= 0) {
		C = 1;
		if (Sin < Cos) 	{ 	//octant = 0;
			X = Sin;
			Y = Cos;
			XY = 32767*X;
			XY = XY/Y;
			A = XY;
			Atan_interp();
			B = A + 0;
		}
		else {		 	//octant = 1;
			X = Cos;
			Y = Sin;
			XY = 32767*X;
			XY = XY/Y;
			A = XY;
			Atan_interp();
			B = 16383 - A;
		}	
	}
	else { //Sin>0 e Cos <0 
		C = 0;
		if (Sin <= -Cos) { 	//octant = 3;
			X = Sin;
			Y = -Cos;
			XY = 32767*X;
			XY = XY/Y;
			A = XY;
			Atan_interp();
			B = 32767 - A;
		}
		else {			//octant = 2;
			X = -Cos;
			Y = Sin;
			XY = 32767*X;
			XY = XY/Y;
			A = XY;
			Atan_interp();
			B = A + 16383;
		}	
	}
}
else {
	S = 0;
	if (Cos >= 0) {	//Sin<0 e Cos>0
		C = 1;
		if (-Sin <= Cos) {	//octant = 7;
			X = -Sin;
			Y = Cos;
			XY = 32767*X;
			XY = XY/Y;
			A = XY;
			Atan_interp();
			B = 32767 + 32767 - A;
		}
		else {			//octant = 6;
			
			X = Cos;
			Y = -Sin;
			XY = 32767*X;
			XY = XY/Y;
			A = XY;
			Atan_interp();
  			B = 32767 + 16383 + A;
		}	
	}
	else { //Sin<0 e Cos<00
		C = 0;
		if (-Sin < -Cos) {	//octant = 4;
			X = -Sin;
			Y = -Cos;
			XY = 32767*X;
			XY = XY/Y;
			A = XY;
			Atan_interp();
			B = 32767 + A;
		}
		else 	{		//octant = 5;
			X = -Cos;
			Y = -Sin;
			XY = 32767*X;
			XY = XY/Y;
			A = XY;
			Atan_interp();
  			B = 32767 + 16383 - A;
		}	
	}	
}
	//A = Octant separated angle
	//B = Entire period angle
	//S/C = Sin/Cos signal	Sa/Ca= Sin/Cos signal before

	//Quadrature Counter/ Course angle calculation 512 counts/mechanical turn
	if (S != Sa) {
		if (S != C) {			thetaI--; Theta_Turns--;		}
		else {			thetaI++;	Theta_Turns++;	}
	}
	if (C != Ca) {
		if (S == C) {			thetaI--;	Theta_Turns--;	}
		else {			thetaI++;	Theta_Turns++;		}
	}
	Sa = S;
	Ca = C;

	//thetaI = Quadrature Counter 4 counts/period -> (128(SKS36) * 4) = 512 counts/mechanical turn
	if(thetaI>511) { 
		thetaI = 0;
		Turns++;
	}
	else if (thetaI <0) {
		thetaI = 511;
		Turns--;
	}
	//Turns = Mechanical turns
	
	//A = (A/512) + 64;
	//thetaI = abs(thetaI)%512;
	//thetaI = 0;
	//A = -32760;
	tets = (thetaI)/4;
	B2 = ((B>>9) + tets*128 + octant*8)%16384;// /128    - B3;	//B3 = Zeramento do Angulo 
	B3 = (B2*4)%16384;
	theta2 = (float) B3;
	theta_e= theta2/(2607.59458f);		//Angulação elétrica de 0 a 2pi ; 1/4 de volta mecanica
	theta2 = (float) B2;
	theta = theta2;///(2607.59458f);			//Angulação mecanica de 0 a 2pi ; 1 volta mecanica
	
	//Calculo theta absoluto
		long int T,T2,temps;
		tets = (Theta_Turns);///4;
	
		if (tets < 0) {
			tets = tets + 1;
			tets = tets/4;
			T = (tets*128);
			theta2 = (float) (128-(B>>9))- octant*8;
			AbsTheta2 = (float) T - theta2;//			FALHA EM angulos negativos (B>>9) +Theta_Turns
		}
		else {
			tets = tets/4;
			
			T = (tets*128);
			theta2 = (float) ((B>>9)+ octant*8);
			AbsTheta2 = (float) T + theta2;//			FALHA EM angulos negativos (B>>9) +Theta_Turns
		}

		

		//AbsTheta2 = AbsTheta2 + ((float) T2/(45.511110f));
		//theta = ((float) thetaI)/4 + theta/128;
		//DAC2->DHR12R1 = B2/4;//2048 + (int)id
		//theta = 2.8125f*theta;
		//theta = theta/360.0f;
		//AbsTheta
	
 	//octant2 = octant;
}

```    
## Still in development

