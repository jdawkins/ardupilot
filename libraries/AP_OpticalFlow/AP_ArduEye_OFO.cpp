
#include "AP_ArduEye_OFO.h"

//class instance to be referenced in sketch
//ArduEyeOFOClass ArduEyeOFO;


/*********************************************************************/
//	LPF
//	Send in a new optical flow measurement and the filtered optical
//	flow value will be changed by ((*new_OF)-(*filtered_OF))*alpha
//	alpha should be between 0-1 
/*********************************************************************/

void ArduEyeOFO::LPF(short *filtered_OF,short *new_OF,float alpha)
{
	(*filtered_OF)=(*filtered_OF)+((float)(*new_OF)-(*filtered_OF))				*alpha;
}

/*********************************************************************/
//	Accumulate
//	The current optical flow value is added to the accumulation sum
//	only if it crosses a threshold
/*********************************************************************/      
short ArduEyeOFO::Accumulate(short *new_OF,short *acc_OF, short threshold)
{
    short reset=0;

    if((*new_OF>threshold)||(*new_OF<-threshold))
    {
        *acc_OF+=*new_OF;
        reset=1;
    }

    return reset;
}


/*********************************************************************/
//	IIA_1D (short version)
//	This is a one dimensional version of the image interpolation
//	algorithm (IIA) as described by Prof. Mandyam Srinivasan. 
//	curr_img and last_img are input line images. numpix is the number //	of pixels in the image. scale is a scaling factor, mostly for the //	benefit of the fixed-poing arithmetic performed here. out is the //	resulting output optical flow.
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	scale: value of one pixel of motion (for scaling output)
//	out: pointer to integer value for output.
/*********************************************************************/

void ArduEyeOFO::IIA_1D(short *curr_img, short *last_img, char numpix, short 					scale, short *out) 
{
  short *pleft,*pright,*pone,*ptwo;
  long top,bottom;
  char i;
  int deltat,deltax;
  
  // Set up pointers
  pleft = curr_img;	//left-shifted image
  pone = curr_img+1;	//center image
  pright = curr_img+2;	//right-shifted image
  ptwo = last_img+1;	//time-shifted image
  
  top=bottom=0;
  
  for(i=0; i<numpix-2; ++i) 
  {
    deltat = *ptwo - *pone;    // temporal gradient i.e. pixel change 					 //over time
    deltax = *pright - *pleft; // spatial gradient i.e. pixel 						 //change over space
    
    top += deltat * deltax;		//top summation
    bottom += deltax * deltax;	//bottom summation

    // Increment pointers
    pleft++;
    pone++;
    pright++;
    ptwo++;
  }
  
  // Compute final output. Note use of "scale" here to multiply 2*top       
  // to a larger number so that it may be meaningfully divided using   
  // fixed point arithmetic
  *out = 2*top*scale/bottom;
}

/*********************************************************************/
//	IIA_1D (char version)
//	This is a one dimensional version of the image interpolation
//	algorithm (IIA) as described by Prof. Mandyam Srinivasan. 
//	curr_img and last_img are input line images. numpix is the number //	of pixels in the image. scale is a scaling factor, mostly for the //	benefit of the fixed-poing arithmetic performed here. out is the //	resulting output optical flow.
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	scale: value of one pixel of motion (for scaling output)
//	out: pointer to integer value for output.
/*********************************************************************/

void ArduEyeOFO::IIA_1D(char *curr_img, char *last_img, char 				numpix, short scale, short *out) 
{
  char *pleft,*pright,*pone,*ptwo;
  long top,bottom;
  int deltat,deltax;
  char i;
  
  // Set up pointers
  pleft = curr_img;	//left-shifted image
  pone = curr_img+1;	//center image
  pright = curr_img+2;	//right-shifted image
  ptwo = last_img+1;	//time-shifted image
  
  top=bottom=0;
  
  for(i=0; i<numpix-2; ++i) 
  {
    deltat = (int)(*ptwo) - (int)(*pone); // temporal gradient i.e. 							// pixel change over time
    deltax = (int)(*pright) - (int)(*pleft); // spatial gradient i.e. 							//pixel change over space
    top += deltat * deltax;
    bottom += deltax * deltax;

    // Increment pointers
    pleft++;
    pone++;
    pright++;
    ptwo++;
  }
  
  // Compute final output. Note use of "scale" here to multiply 2*top   
  // to a larger number so that it may be meaningfully divided using 
  // fixed point arithmetic
  *out = 2*top*scale/bottom;
}


/*********************************************************************/
//	IIA_Plus_2D (char version)
//	This function computes optical flow between two images curr_img //	and last_img using a simplified version of Mandyam Srinivasan's //	image interpolation algorithm. This algorithm assumes that 
//	displacements are generally on the order of one pixel or less. 
//
//	Credit- Thanks to "A.J." on Embedded Eye for optimizing this 
//	function.
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void ArduEyeOFO::IIA_Plus_2D(char *curr_img, char *last_img, short 						rows,short cols, short scale, short 						*ofx, short *ofy)
{
  int32_t  A=0, BD=0, C=0, E=0, F=0;
  int16_t  F2F1, F4F3, FCF0;
        
  // set up pointers
  char *f0 = curr_img + cols + 1; //center image
  char *f1 = curr_img + cols + 2; //right-shifted image
  char *f2 = curr_img + cols;     //left-shifted image	
  char *f3 = curr_img + 2*cols + 1; //down-shifted image	
  char *f4 = curr_img + 1;		//up-shifted image
  char *fz = last_img + cols + 1; 	//time-shifted image


  // loop through
  for (char r=1; r<rows-1; ++r) 
  { 
    for (char c=1; c<cols-1; ++c) 
    { 
      // compute differentials, then increment pointers 
      F2F1 = (*(f2++) - *(f1++));
      F4F3 = (*(f4++) - *(f3++));
      FCF0 = (*(fz++) - *(f0++));

      // update summations
      A += (F2F1 * F2F1);
      BD += (F4F3 * F2F1);
	C  += (FCF0 * F2F1);                   
      E += (F4F3 * F4F3);
      F  += (FCF0 * F4F3);                                   
    }

    f0+=2;	//move to next row of image
    fz+=2;
    f1+=2;
    f2+=2;
    f3+=2;
    f4+=2;
  }
     
  int64_t top1=( (int64_t)(C)*E - (int64_t)(F)*BD );
  int64_t top2=( (int64_t)(A)*F - (int64_t)(C)*BD );
  int64_t bottom=( (int64_t)(A)*E - (int64_t)(BD)*BD );

  // Compute final output. Note use of "scale" here to multiply 2*top   
  // to a larger number so that it may be meaningfully divided using 
  // fixed point arithmetic
  int64_t XS = (2*scale*top1)/bottom;
  int64_t YS = (2*scale*top2)/bottom;
        
  (*ofx) = (short)XS;
  (*ofy) = (short)YS;
}


/*********************************************************************/
//	IIA_Plus_2D (short version)
//	This function computes optical flow between two images curr_img //	and last_img using a simplified version of Mandyam Srinivasan's //	image interpolation algorithm. This algorithm assumes that 
//	displacements are generally on the order of one pixel or less. 
//
//	Credit- Thanks to "A.J." on Embedded Eye for optimizing this 
//	function.
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void ArduEyeOFO::IIA_Plus_2D(short *curr_img, short *last_img, 						short rows,short cols, short scale, 						short *ofx, short *ofy)
{
  int32_t  A=0, BD=0, C=0, E=0, F=0;
  int16_t  F2F1, F4F3, FCF0;
        
  // set up pointers
  short *f0 = curr_img + cols + 1; //center image
  short *f1 = curr_img + cols + 2; //right-shifted image
  short *f2 = curr_img + cols;     //left-shifted image	
  short *f3 = curr_img + 2*cols + 1; //down-shifted image	
  short *f4 = curr_img + 1;		//up-shifted image
  short *fz = last_img + cols + 1; 	//time-shifted image


  // loop through
  for (char r=1; r<rows-1; ++r) 
  { 
    for (char c=1; c<cols-1; ++c) 
    { 
      // compute differentials, then increment pointers 
      F2F1 = (*(f2++) - *(f1++));
      F4F3 = (*(f4++) - *(f3++));
      FCF0 = (*(fz++) - *(f0++));

      // update summations
      A += (F2F1 * F2F1);
      BD += (F4F3 * F2F1);
	C  += (FCF0 * F2F1);                   
      E += (F4F3 * F4F3);
      F  += (FCF0 * F4F3);                                   
    }

    f0+=2;	//move to next row of image
    fz+=2;
    f1+=2;
    f2+=2;
    f3+=2;
    f4+=2;
  }
     
  int64_t top1=( (int64_t)(C)*E - (int64_t)(F)*BD );
  int64_t top2=( (int64_t)(A)*F - (int64_t)(C)*BD );
  int64_t bottom=( (int64_t)(A)*E - (int64_t)(BD)*BD );

  // Compute final output. Note use of "scale" here to multiply 2*top   
  // to a larger number so that it may be meaningfully divided using 
  // fixed point arithmetic
  int64_t XS = (2*scale*top1)/bottom;
  int64_t YS = (2*scale*top2)/bottom;
        
  (*ofx) = (short)XS;
  (*ofy) = (short)YS;
}

/*********************************************************************/
//	IIA_Plus_2D (short version)
//	This function computes optical flow between two images curr_img //	and last_img using a simplified version of Mandyam Srinivasan's //	image interpolation algorithm. This algorithm assumes that 
//	displacements are generally on the order of one pixel or less. 
//
//	Credit- Thanks to "A.J." on Embedded Eye for optimizing this 
//	function.
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void ArduEyeOFO::IIA_Plus_2D(float *curr_img, float *last_img, 						short rows,short cols, short scale, 						short *ofx, short *ofy)
{
  int32_t  A=0, BD=0, C=0, E=0, F=0;
  int16_t  F2F1, F4F3, FCF0;
        
  // set up pointers
  float *f0 = curr_img + cols + 1; //center image
  float *f1 = curr_img + cols + 2; //right-shifted image
  float *f2 = curr_img + cols;     //left-shifted image	
  float *f3 = curr_img + 2*cols + 1; //down-shifted image	
  float *f4 = curr_img + 1;		//up-shifted image
  float *fz = last_img + cols + 1; 	//time-shifted image


  // loop through
  for (char r=1; r<rows-1; ++r) 
  { 
    for (char c=1; c<cols-1; ++c) 
    { 
      // compute differentials, then increment pointers 
      F2F1 = (*(f2++) - *(f1++));
      F4F3 = (*(f4++) - *(f3++));
      FCF0 = (*(fz++) - *(f0++));

      // update summations
      A += (F2F1 * F2F1);
      BD += (F4F3 * F2F1);
	C  += (FCF0 * F2F1);                   
      E += (F4F3 * F4F3);
      F  += (FCF0 * F4F3);                                   
    }

    f0+=2;	//move to next row of image
    fz+=2;
    f1+=2;
    f2+=2;
    f3+=2;
    f4+=2;
  }
     
  int64_t top1=( (int64_t)(C)*E - (int64_t)(F)*BD );
  int64_t top2=( (int64_t)(A)*F - (int64_t)(C)*BD );
  int64_t bottom=( (int64_t)(A)*E - (int64_t)(BD)*BD );

  // Compute final output. Note use of "scale" here to multiply 2*top   
  // to a larger number so that it may be meaningfully divided using 
  // fixed point arithmetic
  int64_t XS = (2*scale*top1)/bottom;
  int64_t YS = (2*scale*top2)/bottom;
        
  (*ofx) = (short)XS;
  (*ofy) = (short)YS;
}
/*********************************************************************/
//	IIA_Square_2D (char version)
//	This function computes optical flow between two images curr_img //	and last_img using a simplified version of Mandyam Srinivasan's //	image interpolation algorithm. This algorithm assumes that 
//	displacements are generally on the order of one pixel or less. 
//	Instead of the traditional 'plus'
//	image shifts (top, bottom, left, right) here we do a more compact
//	square shift (top right, top left, bottom right, bottom left)
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void ArduEyeOFO::IIA_Square_2D(char *curr_img, char *last_img, 						short rows,short cols, short scale, 						short *ofx, short *ofy)
{
  int32_t  A=0, BD=0, C=0, E=0, F=0;
  int16_t  F2F1, F4F3, FCF0;
          
  // set up pointers
  char *f0 = curr_img;			//top left 
  char *f1 = curr_img + 1; 		//top right
  char *f2 = curr_img + cols; 	//bottom left
  char *f3 = curr_img + cols + 1; 	//bottom right
  char *fz = last_img; 		//top left time-shifted

  // loop through
  for (char r=0; r<rows-1; ++r) 
  { 
    for (char c=0; c<cols-1; ++c) 
    { 
      // compute differentials
      F2F1 = ((*(f0) - *(f1)) + (*(f2) - *(f3))) ;
      F4F3 = ((*(f0) - *(f2)) + (*(f1) - *(f3))) ;
      FCF0 = (*(fz) - *(f0));

      //increment pointers
	f0++;
    	fz++;
    	f1++;
    	f2++;
    	f3++;

      // update summations
      A += (F2F1 * F2F1);
      BD += (F4F3 * F2F1);
	C  += (FCF0 * F2F1);                   
      E += (F4F3 * F4F3);
      F  += (FCF0 * F4F3);                                   
    }
    
    //go to next row
    f0++;
    fz++;
    f1++;
    f2++;
    f3++;
  }
       
  int64_t top1=( (int64_t)(C)*E - (int64_t)(F)*BD );
  int64_t top2=( (int64_t)(A)*F - (int64_t)(C)*BD );
  int64_t bottom=( (int64_t)(A)*E - (int64_t)(BD)*BD );

  // Compute final output. Note use of "scale" here to multiply 2*top   
  // to a larger number so that it may be meaningfully divided using 
  // fixed point arithmetic
  int64_t XS = (2*scale*top1)/bottom;
  int64_t YS = (2*scale*top2)/bottom;
        
  (*ofx) = (short)XS;
  (*ofy) = (short)YS;
}

/*********************************************************************/
//	IIA_Square_2D (short version)
//	This function computes optical flow between two images curr_img //	and last_img using a simplified version of Mandyam Srinivasan's //	image interpolation algorithm. This algorithm assumes that 
//	displacements are generally on the order of one pixel or less. 
//	Instead of the traditional 'plus'
//	image shifts (top, bottom, left, right) here we do a more compact
//	square shift (top right, top left, bottom right, bottom left)
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void ArduEyeOFO::IIA_Square_2D(short *curr_img,short *last_img, 						short rows,short cols, short scale, 						short *ofx, short *ofy)
{
  int32_t  A=0, BD=0, C=0, E=0, F=0;
  int16_t  F2F1, F4F3, FCF0;
          
  // set up pointers
  short *f0 = curr_img;			//top left 
  short *f1 = curr_img + 1; 		//top right
  short *f2 = curr_img + cols; 	//bottom left
  short *f3 = curr_img + cols + 1; 	//bottom right
  short *fz = last_img; 		//top left time-shifted

  // loop through
  for (char r=0; r<rows-1; ++r) 
  { 
    for (char c=0; c<cols-1; ++c) 
    { 
      // compute differentials
      F2F1 = ((*(f0) - *(f1)) + (*(f2) - *(f3))) ;
      F4F3 = ((*(f0) - *(f2)) + (*(f1) - *(f3))) ;
      FCF0 = (*(fz) - *(f0));

      //increment pointers
	f0++;
    	fz++;
    	f1++;
    	f2++;
    	f3++;

      // update summations
      A += (F2F1 * F2F1);
      BD += (F4F3 * F2F1);
	C  += (FCF0 * F2F1);                   
      E += (F4F3 * F4F3);
      F  += (FCF0 * F4F3);                                   
    }
    
    //go to next row
    f0++;
    fz++;
    f1++;
    f2++;
    f3++;
  }
       
  int64_t top1=( (int64_t)(C)*E - (int64_t)(F)*BD );
  int64_t top2=( (int64_t)(A)*F - (int64_t)(C)*BD );
  int64_t bottom=( (int64_t)(A)*E - (int64_t)(BD)*BD );

  // Compute final output. Note use of "scale" here to multiply 2*top   
  // to a larger number so that it may be meaningfully divided using 
  // fixed point arithmetic
  int64_t XS = (2*scale*top1)/bottom;
  int64_t YS = (2*scale*top2)/bottom;
        
  (*ofx) = (short)XS;
  (*ofy) = (short)YS;
}

/*********************************************************************/
//	LK_Plus_2D (char version)
//	This function computes optical flow between two images curr_img //	and last_img using a version of Lucas-Kanade's algorithm
//	This algorithm assumes that displacements are generally on 
//	the order of one pixel or less. 
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void ArduEyeOFO::LK_Plus_2D(char *curr_img, char *last_img, short 					   rows,short cols, short scale, short 					   *ofx, short *ofy)
{
  int32_t  A11=0, A12=0, A22=0, b1=0, b2=0;
  int16_t  F2F1, F4F3, FCF0;
        
  // set up pointers
  char *f0 = curr_img + cols + 1; //center image
  char *f1 = curr_img + cols + 2; //right-shifted image
  char *f2 = curr_img + cols;     //left-shifted image	
  char *f3 = curr_img + 2*cols + 1; //down-shifted image	
  char *f4 = curr_img + 1;		//up-shifted image
  char *fz = last_img + cols + 1; 	//time-shifted image

  // loop through
  for (char r=1; r<rows-1; ++r) { 
    for (char c=1; c<cols-1; ++c) { 
      // compute differentials, then increment pointers (post 		// increment)
      F2F1 = (*(f2++) - *(f1++));	//horizontal differential
      F4F3 = (*(f4++) - *(f3++));	//vertical differential
      FCF0 = (*(fz++) - *(f0++));	//time differential

      // update summations
      A11 += (F2F1 * F2F1);
      A12 += (F4F3 * F2F1);
      A22 += (F4F3 * F4F3);
      b1  += (FCF0 * F2F1);                   
      b2  += (FCF0 * F4F3);                                   
    }
    f0+=2;	//move to next row of image
    fz+=2;
    f1+=2;
    f2+=2;
    f3+=2;
    f4+=2;
  }
       
  //determinant
  int64_t detA = ( (int64_t)(A11)*A22 - (int64_t)(A12)*A12 );

  // Compute final output. Note use of "scale" here to multiply 2*top   
  // to a larger number so that it may be meaningfully divided using 
  // fixed point arithmetic
  int64_t XS = ( (int64_t)(b1)*A22 - (int64_t)(b2)*A12 ) * scale / 		detA;
  int64_t YS = ( (int64_t)(b2)*A11 - (int64_t)(b1)*A12 ) * scale / 		detA;
        
  (*ofx) = (short)XS;
  (*ofy) = (short)YS;
}

/*********************************************************************/
//	LK_Plus_2D (short version)
//	This function computes optical flow between two images curr_img //	and last_img using a version of Lucas-Kanade's algorithm
//	This algorithm assumes that displacements are generally on 
//	the order of one pixel or less. 
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void ArduEyeOFO::LK_Plus_2D(short *curr_img, short *last_img, 					   short rows,short cols, short scale, 					   short *ofx, short *ofy)
{
  int32_t  A11=0, A12=0, A22=0, b1=0, b2=0;
  int16_t  F2F1, F4F3, FCF0;
        
  // set up pointers
  short *f0 = curr_img + cols + 1; //center image
  short *f1 = curr_img + cols + 2; //right-shifted image
  short *f2 = curr_img + cols;     //left-shifted image	
  short *f3 = curr_img + 2*cols + 1; //down-shifted image	
  short *f4 = curr_img + 1;		//up-shifted image
  short *fz = last_img + cols + 1; 	//time-shifted image

  // loop through
  for (char r=1; r<rows-1; ++r) { 
    for (char c=1; c<cols-1; ++c) { 
      // compute differentials, then increment pointers (post 		// increment)
      F2F1 = (*(f2++) - *(f1++));	//horizontal differential
      F4F3 = (*(f4++) - *(f3++));	//vertical differential
      FCF0 = (*(fz++) - *(f0++));	//time differential

      // update summations
      A11 += (F2F1 * F2F1);
      A12 += (F4F3 * F2F1);
      A22 += (F4F3 * F4F3);
      b1  += (FCF0 * F2F1);                   
      b2  += (FCF0 * F4F3);                                   
    }
    f0+=2;	//move to next row of image
    fz+=2;
    f1+=2;
    f2+=2;
    f3+=2;
    f4+=2;
  }
       
  //determinant
  int64_t detA = ( (int64_t)(A11)*A22 - (int64_t)(A12)*A12 );

  // Compute final output. Note use of "scale" here to multiply 2*top   
  // to a larger number so that it may be meaningfully divided using 
  // fixed point arithmetic
  int64_t XS = ( (int64_t)(b1)*A22 - (int64_t)(b2)*A12 ) * scale / 		detA;
  int64_t YS = ( (int64_t)(b2)*A11 - (int64_t)(b1)*A12 ) * scale / 		detA;
        
  (*ofx) = (short)XS;
  (*ofy) = (short)YS;
}

/*********************************************************************/
//	LK_Square_2D (char version)
//	This function computes optical flow between two images curr_img //	and last_img using a version of Lucas-Kanade's algorithm
//	This algorithm assumes that displacements are generally on 
//	the order of one pixel or less. Instead of the traditional 'plus'
//	image shifts (top, bottom, left, right) here we do a more compact
//	square shift (two right minus left, two top minus bottom)
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void ArduEyeOFO::LK_Square_2D(char *curr_img, char *last_img, 						short rows,short cols, short scale, 						short *ofx, short *ofy)
{
  int32_t  A11=0, A12=0, A22=0, b1=0, b2=0;
  int16_t  F2F1, F4F3, FCF0;
         
  // set up pointers
  char *f0 = curr_img;			//top left 
  char *f1 = curr_img + 1; 		//top right
  char *f2 = curr_img + cols; 	//bottom left
  char *f3 = curr_img + cols + 1; 	//bottom right
  char *fz = last_img; 		//top left time-shifted



  // loop through
  for (char r=0; r<rows-1; ++r) 
  { 
    for (char c=0; c<cols-1; ++c) 
    { 
      // compute differentials      
	F2F1 = ((*(f0) - *(f1)) + (*(f2) - *(f3))) ;
      F4F3 = ((*(f0) - *(f2)) + (*(f1) - *(f3))) ;
      FCF0 = (*(fz) - *(f0));

	//increment pointers
	f0++;
    	fz++;
    	f1++;
    	f2++;
    	f3++;

      // update summations
      A11 += (F2F1 * F2F1);
      A12 += (F4F3 * F2F1);
      A22 += (F4F3 * F4F3);
      b1  += (FCF0 * F2F1);                   
      b2  += (FCF0 * F4F3);                                   
    }

    //go to next row
    f0++;
    fz++;
    f1++;
    f2++;
    f3++;
  }
       
  //compute determinant
  int64_t detA = ( (int64_t)(A11)*A22 - (int64_t)(A12)*A12 );

  // Compute final output. Note use of "scale" here to multiply 2*top   
  // to a larger number so that it may be meaningfully divided using 
  // fixed point arithmetic
  int64_t XS = ( (int64_t)(b1)*A22 - (int64_t)(b2)*A12 ) * scale / 		detA;
  int64_t YS = ( (int64_t)(b2)*A11 - (int64_t)(b1)*A12 ) * scale / 		detA;
        
  (*ofx) = (short)XS;
  (*ofy) = (short)YS;
}

/*********************************************************************/
//	LK_Square_2D (short version)
//	This function computes optical flow between two images curr_img //	and last_img using a version of Lucas-Kanade's algorithm
//	This algorithm assumes that displacements are generally on 
//	the order of one pixel or less. Instead of the traditional 'plus'
//	image shifts (top, bottom, left, right) here we do a more compact
//	square shift (top right, top left, bottom right, bottom left)
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void ArduEyeOFO::LK_Square_2D(short *curr_img, short *last_img, 						short rows, short cols, short 						scale, short *ofx, short *ofy)
{
  int32_t  A11=0, A12=0, A22=0, b1=0, b2=0;
  int16_t  F2F1, F4F3, FCF0;
        
  // set up pointers
  short *f0 = curr_img;			//top left 
  short *f1 = curr_img + 1; 		//top right
  short *f2 = curr_img + cols; 	//bottom left
  short *f3 = curr_img + cols + 1; 	//bottom right
  short *fz = last_img; 		//top left time-shifted


  // loop through
  for (char r=0; r<rows-1; ++r) 
  { 
    for (char c=0; c<cols-1; ++c) 
    { 
      // compute differentials
      F2F1 = (*(f0) - *(f1)) + (*(f2) - *(f3))  ;
      F4F3 = (*(f0) - *(f2)) + (*(f1) - *(f3))  ;
      FCF0 = (*(fz) - *(f0));
	
      //increment pointers
	f0++;
    	fz++;
    	f1++;
    	f2++;
    	f3++;

      // update summations
      A11 += (F2F1 * F2F1);
      A12 += (F4F3 * F2F1);
      A22 += (F4F3 * F4F3);
      b1  += (FCF0 * F2F1);                   
      b2  += (FCF0 * F4F3);                                   
    }
  
    //go to next row
    f0++;
    fz++;
    f1++;
    f2++;
    f3++;
  }
       
  //compute determinant
  int64_t detA = ( (int64_t)(A11)*A22 - (int64_t)(A12)*A12 );

  // Compute final output. Note use of "scale" here to multiply 2*top   
  // to a larger number so that it may be meaningfully divided using 
  // fixed point arithmetic
  int64_t XS = ( (int64_t)(b1)*A22 - (int64_t)(b2)*A12 ) * scale / 			detA;
  int64_t YS = ( (int64_t)(b2)*A11 - (int64_t)(b1)*A12 ) * scale / 			detA;
        
  (*ofx) = (short)XS;
  (*ofy) = (short)YS;
}
