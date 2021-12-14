//ENCODER VARIABLES

int indexx;						//Interpolation index variable
long int interp;					//Interpolation calc
int A;							//Input for the interpolation function
int Sin,Cos;						//Angle sector calculation
int X,Y;						//Angle sector calculation
long int XY;						//Angle sector calculation
int Sa,Ca,S,C;						//Quadrature calculation, like a normal pulsed encoder
long int thetaI,Turns,Theta_Turns;			//Multi turn calculation
uint16_t B,B2,B3;					//Multi turn calculation
long int tets;						//Multi turn calculation
float theta,theta2,theta_e,AbsTheta,AbsTheta2;		//Multi turn calculation	
int octant,octant2;					//Angle octant 
long int T,T2,temps;



//Analog acquisition inside the ADC OnEnd interruption function (not showed here) saves in Sin and Cos variables and run Enc_Process()



int ATAN_Table[33] = 
{
58,331,653,976,1298,1617,1934,2247,2555,2860,3159,3453,3742,4025,4301,4572,
4836,5093,5344,5588,5826,6057,6282,6500,6712,6917,7116,7310,7497,7679,7855,8026,8192
};

void Atan_interp(void) //ArcTan interpolation
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
	
if (Sin >= 0) 
{
	S = 1;
	if (Cos >= 0) {
		C = 1;
		if (Sin < Cos) 	{ 	
			octant = 0;
			X = Sin;
			Y = Cos;
			XY = 32767*X;
			XY = XY/Y;
			A = XY;
			Atan_interp();
			B = A + 0;
		}
		else {		 	
			octant = 1;
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
		if (Sin <= -Cos) { 	
			octant = 3;
			X = Sin;
			Y = -Cos;
			XY = 32767*X;
			XY = XY/Y;
			A = XY;
			Atan_interp();
			B = 32767 - A;
		}
		else {			
			octant = 2;
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
		if (-Sin <= Cos) {	
			octant = 7;
			X = -Sin;
			Y = Cos;
			XY = 32767*X;
			XY = XY/Y;
			A = XY;
			Atan_interp();
			B = 32767 + 32767 - A;
		}
		else {			
			octant = 6;
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
		if (-Sin < -Cos) {	
			octant = 4;
			X = -Sin;
			Y = -Cos;
			XY = 32767*X;
			XY = XY/Y;
			A = XY;
			Atan_interp();
			B = 32767 + A;
		}
		else 	{		
			octant = 5;
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

	//Quadrature Counter	
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

	//thetaI = Quadrature Counter 4 counts/period, obviously -> (128(SKS36) * 4) = 512 counts/mechanical turn
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
