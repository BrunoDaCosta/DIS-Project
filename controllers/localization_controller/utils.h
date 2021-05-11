#ifndef UTILS_H
#define UTILS_H 

#define MMS 4 //Matrix max size

void add(double a[][MMS],double b[][MMS],double res[][MMS],int r1,int c1,int r2,int c2)
{
  if (c1!=c2 || r1!=r2){
    printf("Error: Matrix size don't match");
    return;
  }
  
  int i,j;
  for(i=0; i<r1; ++i)
    for(j=0; j<c1; ++j)
      res[i][j]=a[i][j]+b[i][j];
}

void transp(double a[][MMS],double res[][MMS], int r1, int c1){
  int i,j;
  for(i=0; i<r1; ++i)
    for(j=0; j<c1; ++j)
      res[j][i]=a[i][j];
}

void scalar_mult(double scalar, double a[][MMS], double res[][MMS], int r1, int c1){
  int i,j;
  for(i=0; i<r1; ++i)
    for(j=0; j<c1; ++j)
      res[i][j]=scalar*a[i][j];
}

void mult(double a[][MMS],double b[][MMS],double res[][MMS],int r1,int c1,int r2,int c2)
{
    if (c1!=r2){
      printf("Error: Matrix size don't match");
      return;
    }
    
    int i,j,k;
    /* Initializing elements of matrix mult to 0.*/
    for(i=0; i<r2; ++i)
        for(j=0; j<c1; ++j)
            res[i][j]=0;
        
    /* Multiplying matrix a and b and storing in array mult. */
    for(i=0; i<r1; ++i)
        for(j=0; j<c2; ++j)
            for(k=0; k<c1; ++k)
                res[i][j]+=a[i][k]*b[k][j];         
}
 
void inv(double a[][MMS], double res[][MMS]){
  double det=1/(a[0][0]*a[1][1]-a[0][1]*a[1][0]);
  printf("%g\n", det);
  res[0][0]=a[1][1]*det;
  res[1][1]=a[0][0]*det;
  res[1][0]=-a[1][0]*det;
  res[0][1]=-a[0][1]*det;
}
 
void copy_matrix(double a[MMS][MMS], double res[MMS][MMS], int r1, int c1){
  int i,j;
  for(i=0; i<r1; ++i)
    for(j=0; j<c1; ++j)
      res[i][j]=a[i][j];

}


void print_matrix(double a[][MMS], int row, int col)
{
    int i, j;
    for(i=0; i<row; ++i)
        for(j=0; j<col; ++j){
            printf("%g  ",a[i][j]);
            if(j==col-1)
                printf("\n\n");
        }
}


#endif