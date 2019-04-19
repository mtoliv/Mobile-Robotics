#include "mbed.h"
#include "math.h"
#include "Robot.h"
#include "Matrix.h"
#include "MatrixMath.h"

const float PI=3.14159265359;

struct ponto{
float x_coord ;
float y_coord ;
float theta_coord ;
}ponto_actual = {30.0f,30.0f,0.0f},ponto_anterior = {30.0f,30.0f,0.0f};

struct celula_robo{
        float x;
        float y;    
}celula_actual = {0,0};

typedef struct Forcas{
    float componente_x;
    float componente_y;
}forca;

    Serial pc(SERIAL_TX, SERIAL_RX);
    DigitalOut myled1(LED1);
    DigitalIn start_button(USER_BUTTON);
    
    int sector[37] = {0};
    int *sector_ptr = sector;
    int sector_filtered[37] = {0};
    int *sector_filtered_ptr = sector_filtered;
    int circular_sector[3*36+1] = {0};
    int *circular_ptr = circular_sector;
    float beta, b = 2.0f, a = b*sqrt(50.0f);
    int m,k;
    float kv=6.9f;//4.1f;//6.5f;
    float ki=0.3f;//0.1f; 
    float ks=31.8f;//12.0f; 
    
     //-------------------------------------------------------------------//
    celula_robo celula_desejada ;
    float  lim_cell = 5.0f;//numero de celulas que fazem parte da zona activa
    float pontos_por_celula = 5.0f;
    Matrix C(80,80);
    Matrix T(3,3);

    Matrix Za(3,1);
    Matrix robot_pontos(3,1);
    
    const float encoder_resolution=1440;//Resolução dos encoders
    const float RADII=3.5;//Raio das rodas da plataforma
    const float L=14.1;//Comprimento do eixo entre rodas
    const float sample_time=0.1;

    float time_sum = sample_time;
    char rx;
    Timer t;
   //Cricao de objectos
    /*
    float obx1 = 70.0f, oby1= 30.0f;
    float obx2 = 30.0f, oby2=10.0f;
    float obx3 = 30.0f, oby3=65.0f;
    float obdimx1 = 20.0f, obdimy1 = 20.0f;
    float obdimx2 = 10.0f, obdimy2 = 30.0f;  
    float obdimx3 = 15.0f, obdimy3 = 20.0f;  
    */
    float xmin,ymin,xmax,ymax;
    
    float obx1 = 52.5f, oby1= 52.5f;
    float obx2 = 300.5f, oby2=300.5f;
    float obx3 = 300.0f, oby3=300.0f;
    
    float obdimx1 = 20.0f, obdimy1 = 20.0f;
    float obdimx2 = 20.0f, obdimy2 = 50.0f;
    float obdimx3 = 5.0f, obdimy3 = 5.0f;  
  
      
    celula_robo cel_obstaculo= {0,0};
    /*obstaculo.x_coord = obx1;
    obstaculo.y_coord = oby1;*/
    ponto obstaculo = {obx1,oby1};
//-----------FUNCOES----------------
float calcula_erro(float d_desejado, float x_desesjado, float x_medido , float y_desejado, float y_medido);
float calcula_theta(float x_desesjado, float x_medido , float y_desejado, float y_medido);
float theta_diferenca(float theta_desejado, float theta_coord);
void coordenadas_novas(ponto* P_actual,ponto* P_anterior);
void algo(const ponto *desejado_ULTIMATE, float d_desejado);
void calcula_F_direcao(Matrix C,const celula_robo *cell_robot,  forca *resultante);
void forca2ponto(const forca *F_result,const  ponto *actual,ponto *intermedio, float d_desejado);
void ponto2celula(const ponto *actual,celula_robo *cell);
void cria_parede(Matrix C);
void cria_obstaculo(Matrix C);
void filter_sector(int *sector);
int sector_alg(int *sector,int sot);
//------------------------------

int main() {
    myled1 = 0;
    float aux;
    T.add(1,1,0);T.add(1,2,-1);T.add(1,3,80);//
    T.add(2,1,1);T.add(2,2,0);T.add(2,3,1);// T AINDA NÃO ESTÁ CERTO
    T.add(3,1,0);T.add(3,2,0);T.add(3,3,1);//
    
    ponto2celula(&obstaculo,&cel_obstaculo);
    xmin=cel_obstaculo.x;
    ymin=cel_obstaculo.y;
    obstaculo.x_coord = obx1+obdimx1;
    obstaculo.y_coord = oby1+obdimy1;
    ponto2celula(&obstaculo,&cel_obstaculo);
    xmax=cel_obstaculo.x;
    ymax=cel_obstaculo.y;

     for (aux=xmax;aux<=xmin;aux++){

         C.add(aux,ymin,1);
         C.add(aux,ymax,1);
     }
      for (aux=ymin;aux<=ymax;aux++){

         C.add(xmin,aux,1);
         C.add(xmax,aux,1);
      }

    ponto obstaculo = {obx2,oby2};
    ponto2celula(&obstaculo,&cel_obstaculo);
    xmin=cel_obstaculo.x;
    ymin=cel_obstaculo.y;
    obstaculo.x_coord = obx2+obdimx2;
    obstaculo.y_coord = oby2+obdimy2;
    ponto2celula(&obstaculo,&cel_obstaculo);
    xmax=cel_obstaculo.x;
    ymax=cel_obstaculo.y;

     for (aux=xmax;aux<=xmin;aux++){
         C.add(aux,ymin,1);
         C.add(aux,ymax,1);
     }
      for (aux=ymin;aux<=ymax;aux++){
         C.add(xmin,aux,1);
         C.add(xmax,aux,1);
      }
     ponto obstaculo2 = {obx3,oby3};
    ponto2celula(&obstaculo2,&cel_obstaculo);
    xmin=cel_obstaculo.x;
    ymin=cel_obstaculo.y;
    obstaculo2.x_coord = obx3+obdimx3;
    obstaculo2.y_coord = oby3+obdimy3;
    ponto2celula(&obstaculo2,&cel_obstaculo);
    xmax=cel_obstaculo.x;
    ymax=cel_obstaculo.y;

     for (aux=xmax;aux<=xmin;aux++){
         C.add(aux,ymin,1);
         C.add(aux,ymax,1);
     }
      for (aux=ymin;aux<=ymax;aux++){
         C.add(xmin,aux,1);
         C.add(xmax,aux,1);
      }


    pc.baud(115200);
    float  x_desejado_ULTIMATE=80.0f;
    float  y_desejado_ULTIMATE=80.0f;
    ponto desejado_ULTIMATE = {.x_coord= x_desejado_ULTIMATE , .y_coord = y_desejado_ULTIMATE , .theta_coord =0.0f };
    
    float  d_desejado = 0.0f; //centimetros !!!

    //cria_obstaculo(C);
    int j;
    for (j=1;j<81;j++){
        C.add(j,1,1);
        C.add(j,80,1);
        C.add(1,j,1);
        C.add(80,j,1);
    }

    while(start_button==1){
            setSpeeds(0,0);
    }
    algo(&desejado_ULTIMATE,d_desejado);
    //printf("\nStop");
    setSpeeds(0,0);
    myled1 = 1;
}

//---------------------
void algo(const ponto *desejado_ULTIMATE, float d_desejado){
   float velocidade = 0.0f, omega = 0.0f;
   float omega_E=0.0f,omega_R=0.0f;
   float theta_desejado, theta_diff;
   float erro_actual = 0.0, erro_sum = 0.0,erro_anterior=0.0f;
   forca resultante;
   ponto desejado;
   desejado.x_coord =  ponto_actual.x_coord;
   desejado.y_coord =  ponto_actual.y_coord;
   
   
   ponto2celula(desejado_ULTIMATE ,&celula_desejada);
   ponto2celula(&ponto_actual,&celula_actual);

   

   
    while(1){ // se entrar na celula certa !!!!!!!!!!!!!!!!!!!!
        if(celula_actual.x==celula_desejada.x && celula_actual.y==celula_desejada.y){
                setSpeeds(0,0);
                myled1 = 1;
                while(1);
                }
        t.start();
        getCountsAndReset(); // Call getCountsAndReset() to read the values of encoders
 
        coordenadas_novas(&ponto_actual,&ponto_anterior);
        ponto_anterior.theta_coord=ponto_actual.theta_coord;
        ponto_anterior.x_coord = ponto_actual.x_coord;
        ponto_anterior.y_coord = ponto_actual.y_coord;
        
        pc.printf("%f %f\n", ponto_actual.x_coord,ponto_actual.y_coord);
        
        ponto2celula(&ponto_actual,&celula_actual);
        

        if(celula_actual.x==celula_desejada.x && celula_actual.y==celula_desejada.y){
                setSpeeds(0,0);
                myled1 = 1;
                while(1);
                }

        calcula_F_direcao(C,&celula_actual, & resultante);  
       
        forca2ponto(&resultante,& ponto_actual,&desejado,d_desejado);
 

        //----------------------------------Velocidade para rodas Kv e Ki -------------------------------------
        erro_anterior = erro_actual;
        erro_actual = calcula_erro(d_desejado,desejado.x_coord,ponto_actual.x_coord,desejado.y_coord,ponto_actual.y_coord);
        //pc.printf("\nerro_actual :%f",erro_actual);             
        erro_sum += erro_actual;
        //erro_sum=sample_time*(erro_anterior+erro_actual)/2;
        //pc.printf("\nerro_sum :%f",erro_sum);   
        velocidade = kv*erro_actual + ki*erro_sum;
       // pc.printf("\nvelocidade :%f",velocidade);   
        theta_desejado = calcula_theta(desejado.x_coord,ponto_actual.x_coord,desejado.y_coord,ponto_actual.y_coord);
       // pc.printf("\ntheta_desejado :%f",theta_desejado);   
        theta_diff = theta_diferenca(theta_desejado,ponto_actual.theta_coord);
       // pc.printf("\ntheta_diff :%f",theta_diff);   
        omega = ks*(theta_diff); 
      //  pc.printf("\nomega :%f",omega);   
        //----------------------------
        
        omega_E=(velocidade-(L/2)*omega)/RADII;
        omega_R=(velocidade+(L/2)*omega)/RADII;


            setSpeeds(omega_E,omega_R);
            if(celula_actual.x==celula_desejada.x && celula_actual.y==celula_desejada.y){
                setSpeeds(0,0);
                myled1 = 1;
                while(1);
                }

        while(t.read()<sample_time);
        
        t.stop();
        t.reset();
    }

}
//--------------------------------------------
float calcula_erro(float d_desejado, float x_desejado, float x_medido , float y_desejado, float y_medido){
    float erro_x;
    float erro_y;
    float erro;
    erro_x = x_desejado - x_medido;
 //   pc.printf("\nerro_x:%f",erro_x);   
    erro_y = y_desejado - y_medido;
  //  pc.printf("\nerro_y:%f",erro_y);   
    erro = pow(erro_x,2)+pow(erro_y,2);
    erro = sqrt(erro) - d_desejado;
    return erro;
}
//----------------------------------------
float calcula_theta(float x_desejado, float x_medido , float y_desejado, float y_medido){
    float theta;
    float erro_x;
    float erro_y;
    erro_x = x_desejado - x_medido;
    erro_y = y_desejado - y_medido;
    theta = atan2(erro_y,erro_x);
    return theta;
}
//-----------------------------------------
float theta_diferenca(float theta_desejado, float theta_coord){
    float  theta_aux;
    theta_aux = theta_desejado - theta_coord;
    theta_aux =atan2(sin(theta_aux),cos(theta_aux));
    return theta_aux;
}
//-----------------------------------------------
void coordenadas_novas(ponto* P_actual,ponto* P_anterior){
         float delta_D_R=0.0f,delta_D_E=0.0f;
         float d_o_2=0.0f;
         float delta_D=0.0f, delta_omega = 0.0f;
         
        
        delta_D_E=countsLeft*2*PI*RADII/encoder_resolution;
        delta_D_R=countsRight*2*PI*RADII/encoder_resolution;

        delta_D=(delta_D_E+delta_D_R)/2;
        delta_omega=(delta_D_R-delta_D_E)/L;
      
       
       d_o_2=delta_omega/2.0f; //necesario nos calculos abaixo

        if (delta_omega==0.0f){
            P_actual->x_coord =P_anterior->x_coord+delta_D*cos(P_actual->theta_coord+d_o_2);
            P_actual->y_coord=P_anterior->y_coord+delta_D*sin(P_actual->theta_coord+d_o_2);
            }
        else{
            P_actual->x_coord =P_anterior->x_coord+delta_D*sin(d_o_2)*cos(P_actual->theta_coord+d_o_2)/(d_o_2);
            P_actual->y_coord=P_anterior->y_coord+delta_D*sin(d_o_2)*sin(P_actual->theta_coord+d_o_2)/(d_o_2);
            }    
        P_actual->theta_coord=P_anterior->theta_coord+delta_omega; 
}
//-------------------------------
void calcula_F_direcao(Matrix C,const celula_robo *cell_robot,  forca *resultante){
    int i = 0;
    int j = 0;
    float check=0.0f;
    float dist;
    int chosen_sector,sector_d;
    float chosen_sector_deg;
    //falta meter rotação aqui!
    
    for (i = (cell_robot->x - lim_cell); i <= (cell_robot->x + lim_cell) ; i++){
         for (j = cell_robot->y - lim_cell; j <= cell_robot->y + lim_cell ; j++){
            if(i < 1 || i > 80 || j < 1 || j > 80){
                continue;
            }
            else{
                check=C.getNumber(i,j);
                if (check == 1.0f){
                    dist=sqrt(pow((cell_robot->x-i),2)+pow((cell_robot->y-j),2));
             //       printf("\n Dist %f",dist);
             //       printf("\n obstaculos = %f , y = %f",i,j);
             //       printf("\n cell_robot x = %f , y = %f",cell_robot->x,cell_robot->y);
                    beta=atan2((j-cell_robot->y),(i-cell_robot->x));
                    beta=beta-PI/2;
                    beta=(beta*180.0f)/PI;
                    if (beta<=0.0f){
                        beta+=360.0f;}
                    m = ceil(pow(check,2)*(a - b*dist));
                    k = ceil(beta/10);
                    if (k == 0){
                        k=1;}
                    sector[(int)k]+=m;
                    }
                    
            }
        }
    }     
    filter_sector(sector_ptr);//sector_filtered
  
    
    
    beta=atan2((celula_desejada.y-cell_robot->y),(celula_desejada.x-cell_robot->x));
    
   
    beta=beta-PI/2;
    beta=(beta*180.0f)/PI;
        if (beta<=0.0f){
            beta+=360.0f;}
    k = ceil(beta/10);
        if (k == 0){
            k=1;
        }
    j=1;
    for(i=1;i<=3*36;i++){
            circular_sector[(int)i]=sector_filtered[(int)j];
            j++;
            if (j==37) j=1;
            if (j==73) j=1;
            if (j==109) j=1;
    }
    //chosen sector!!
    //printf("\n k %d",k);
    chosen_sector = sector_alg(circular_ptr,(int)k+36);/*recebe o sector repetido!*/
    chosen_sector_deg = 2.0f*PI*chosen_sector/36.0f;
    chosen_sector_deg = atan2(sin(chosen_sector_deg),cos(chosen_sector_deg));
    while(chosen_sector_deg <=0.0f)
        chosen_sector_deg+=2.0f*PI;
    chosen_sector_deg=chosen_sector_deg*36.0f/(2.0f*PI);
    chosen_sector_deg=floor(chosen_sector_deg);
    sector_d=(int)chosen_sector_deg*10-5;
    if (cell_robot->y>=11.0f){
    //printf("\n c s %f sector %d",chosen_sector_deg,sector_d);
    }
 

    resultante->componente_x=cos(2.0f*PI*(float)sector_d/360.0f)*5.0f;
    resultante->componente_y=sin(2.0f*PI*(float)sector_d/360.0f)*5.0f;
//    pc.printf("\n Forcas T : x %f y %f ",resultante->componente_x,resultante->componente_y);
    for(i=1;i<=36;i++) sector[(int)i]=0;
    for(i=0;i<=36;i++) sector_filtered[(int)i]=0;
    for(i=0;i<=36*3+1;i++) circular_sector[(int)i]=0;
    
}
//-----------------------------------------
void forca2ponto(const forca *F_result,const  ponto *actual,ponto *ponto_intermedio, float d_desejado){
    ponto_intermedio->x_coord = actual->x_coord + F_result->componente_x;
    ponto_intermedio->y_coord = actual->y_coord + F_result->componente_y;
 //   printf(" O ponoto intermeido desejado deu %f em x e % em y nao esquecer que d = %f \n", ponto_intermedio->x_coord, ponto_intermedio->y_coord ,d_desejado);
}
//--------------------------------------------
void ponto2celula(const  ponto *actual,celula_robo *cell){
    cell->x = floor(actual->x_coord / pontos_por_celula);
    cell->y = floor(actual->y_coord/pontos_por_celula);
    //printf("\n celly %f cellx %f",cell->x,cell->y);
    robot_pontos.add(1,1,cell->x);
    robot_pontos.add(2,1,cell->y);
    robot_pontos.add(3,1,1);
    Za=T*robot_pontos;
    cell->x=Za.getNumber(1,1);
    cell->y=Za.getNumber(2,1);
    //printf("\n celly %f cellx %f",cell->x,cell->y);
}
//------------------------------
     
void cria_parede(Matrix C){
    int j;
    
   for (j=1;j<81;j++){
        C.add(j,1,1);
        C.add(j,80,1);
        C.add(1,j,1);
        C.add(80,j,1);
    }
    
    
    }
//---------------------------------
void cria_obstaculo(Matrix C){
    int j;
    
    for (j=30;j<60;j++){
        C.add(50,j,1.0f);
        C.add(70,j,1.0f);
    }

    for (j=50;j<70;j++){
        C.add(j,30,1.0f);
        C.add(j,60,1.0f);
    }
    
}


void filter_sector(int *sector){
 //   float sector_filtered[37] = {0};
        for (k = 1; k<=36;k++){
            if (k == 1)
                sector_filtered[(int)k] = sector[33]+2*sector[34]+3*sector[35]+4*sector[36]+5*sector[(int)k]+4*sector[(int)k+1]+3*sector[(int)k+2]+2*sector[(int)k+3]+sector[(int)k+4];
            else if (k == 2)
                 sector_filtered[(int)k] = sector[(int)34]+2*sector[(int)35]+3*sector[(int)36]+4*sector[(int)k-1]+5*sector[(int)k]+4*sector[(int)k+1]+3*sector[(int)k+2]+2*sector[(int)k+3]+sector[(int)k+4];
            else if (k == 3)
                 sector_filtered[(int)k] = sector[(int)35]+2*sector[(int)36]+3*sector[(int)k-2]+4*sector[(int)k-1]+5*sector[(int)k]+4*sector[(int)k+1]+3*sector[(int)k+2]+2*sector[(int)k+3]+sector[(int)k+4];
            else if (k == 4)
                 sector_filtered[(int)k] = sector[(int)36]+2*sector[(int)k-3]+3*sector[(int)k-2]+4*sector[(int)k-1]+5*sector[(int)k]+4*sector[(int)k+1]+3*sector[(int)k+2]+2*sector[(int)k+3]+sector[(int)k+4]; 
            else if (k == 33)
                 sector_filtered[(int)k] = sector[(int)k-4]+2*sector[(int)k-3]+3*sector[(int)k-2]+4*sector[(int)k-1]+5*sector[(int)k]+4*sector[(int)k+1]+3*sector[(int)k+2]+2*sector[(int)k+3]+sector[(int)1];      
            else if (k == 34)
                 sector_filtered[(int)k] = sector[(int)k-4]+2*sector[(int)k-3]+3*sector[(int)k-2]+4*sector[(int)k-1]+5*sector[(int)k]+4*sector[(int)k+1]+3*sector[(int)k+2]+2*sector[(int)1]+sector[(int)2];      
            else if (k == 35)
                 sector_filtered[(int)k] = sector[(int)k-4]+2*sector[(int)k-3]+3*sector[(int)k-2]+4*sector[(int)k-1]+5*sector[(int)k]+4*sector[(int)k+1]+3*sector[(int)1]+2*sector[(int)2]+sector[(int)3];      
            else if (k == 36)
                 sector_filtered[(int)k] = sector[(int)k-4]+2*sector[(int)k-3]+3*sector[(int)k-2]+4*sector[(int)k-1]+5*sector[(int)k]+4*sector[(int)1]+3*sector[(int)2]+2*sector[(int)3]+sector[(int)4];      
            else
                 sector_filtered[(int)k] = sector[(int)k-4]+2*sector[(int)k-3]+3*sector[(int)k-2]+4*sector[(int)k-1]+5*sector[(int)k]+4*sector[(int)k+1]+3*sector[(int)k+2]+2*sector[(int)k+3]+sector[(int)k+4];      
        }
   

}

//----------------------------------------
int sector_alg(int *sector,int sot){
int chosen_sector_pos,chosen_sector_neg;
int Smax = 14;//wide valley
int POD = 125;
int i=1,j=0;
int check_free=0;
int kv1=0;
int kv2=0;
int chosen_sector=-1;
int NV = 8;
//is path to sector free?
for (i=sot-3;i<=sot+3;i++){
    if (sector[(int)i]<POD)
        check_free+=1;
}

if (check_free == 7){// %target sector is clear of obstacles
    chosen_sector = sot;
    return(chosen_sector);
}


for (i=0;i<3*36;i++){
    kv1=0;
    kv2=0;
    if (sector[(int)(sot+i)]<POD){// %candidate sector
        kv1=sot+i;
        for (j = i+1;j<=i+NV;j++){
            if (sector[(int)(sot+j)]>POD)//%n encontrou sector medio
                break;
            else if (j == i+NV){
                kv2 = sot+j;
                while(sector[(int)(sot+j)]<=POD){
                    kv2 = sot + j;
                    if(j == i + Smax){
                        kv2=sot+j;
                        break;}
                        j++;
                        }
                        }
        }
    }
     
    if ((kv1!=0) && (kv2!=0)){
        chosen_sector_pos=(kv1+kv2)/2;
        break;
    }
}

for (i=0;i<=3*36;i++){
    kv1=0;
    kv2=0;
    if (sector[(int)(sot-i)]<POD){// %candidate sector
        kv1=sot-i;
        for (j = i+1;j<=i+NV;j++){
            if (sector[(int)(sot-j)]>POD)//%n encontrou sector medio
                break;
            else if (j == i+NV){
                kv2 = sot-j;
                while(sector[(int)(sot-j)]<=POD){
                    kv2=sot-j;
                    if (j == i + Smax){
                        kv2 = sot -j;
                        break;}
                        j++;
                        }
                        }
        }
    }
     
    if (kv1!=0 && kv2!=0){
        chosen_sector_neg=(kv1+kv2)/2;
        break;
    }
}

if (abs(chosen_sector_neg-sot)<abs(chosen_sector_pos-sot))
    chosen_sector=chosen_sector_neg;
else
    chosen_sector=chosen_sector_pos;

return((int)chosen_sector);

}
