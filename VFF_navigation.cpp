#include "mbed.h"
#include "math.h"
#include "Robot.h"
#include "Matrix.h"
#include "MatrixMath.h"
#include <vector>

#define LINESIZE 79.0f

const float PI=3.14159265359;
 

struct ponto{
float x_coord ;
float y_coord ;
float theta_coord ;
}ponto_actual = {30.0f,30.0f,0.0f};
struct ponto ponto_anterior = ponto_actual;
    

struct celula_robo{
        float x;
        float y;    
}celula_actual = {0,0};

typedef struct Forcas{
    float componente_x;
    float componente_y;
}forca;


    Serial pc(SERIAL_TX, SERIAL_RX);
    DigitalIn start_button(USER_BUTTON);
    
    
    float x_pusuit; //variaveis pursuit , tipo cenoura para o burro :)
    float y_pusuit;
    
    float kv=6.2f;//4.1f;//6.5f;
    float ki=0.07f;//0.1f; 
    float ks=17.6f;//12.0f; 
    
     //-------------------------------------------------------------------//
     float Fcr = 38.45f ;//Forca Constante repulsiva 
    float Fca = 15.3f;//Forca Constante atrativa 
    float F_aux;
    celula_robo celula_desejada ;
    float  lim_cell = 5.0f;//numero de celulas que fazem parte da zona activa
    float pontos_por_celula = 5.0f;
    Matrix C(80,80);
    Matrix T(3,3);

    Matrix Za(3,1);
    Matrix robot_pontos(3,1);
     //-------------------------------------//
    //-------Calculo de repulaso-----------//
    //-------------------------------------//
    const float encoder_resolution=1440;//Resolução dos encoders
    const float RADII=3.5;//Raio das rodas da plataforma
    const float L=14.1;//Comprimento do eixo entre rodas
    const float sample_time=0.1;   
    float time_sum = sample_time;
    char rx;
    Timer t;


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
//--------------------------------------------------------
    class objecto{
     private:
         float comprimento_y;
        float largura_x;
        ponto ponto_inf_esq;
       
        public:
        objecto( float incio_x, float incio_y,float largura,float  comprimento){
               this->comprimento_y = comprimento;
               this->largura_x = largura;
               this->ponto_inf_esq .x_coord = incio_x;
               this->ponto_inf_esq .y_coord = incio_y;
               this->ponto_inf_esq .theta_coord = 0.0f;
        }
         void add_object(const  Matrix &T,Matrix &C){
                    celula_robo cel_obstaculo= {0,0};
                    float xmin,ymin,xmax,ymax;
                    float aux;
                    
                    ponto2celula(&this->ponto_inf_esq,&cel_obstaculo);
                    xmin=cel_obstaculo.x;
                    ymin=cel_obstaculo.y;
                    ponto_inf_esq.x_coord = this->ponto_inf_esq.x_coord +this->largura_x;
                    ponto_inf_esq.y_coord = this->ponto_inf_esq.y_coord +this->comprimento_y;
                    ponto2celula(&this->ponto_inf_esq,&cel_obstaculo);
                    xmax=cel_obstaculo.x + 1.0f;
                    ymax=cel_obstaculo.y -1.0f;
                    for (aux=xmax;aux<=xmin;aux++){
                           C.add(aux,ymin,1);
                           C.add(aux,ymax,1);
                    }
                    for (aux=ymin;aux<=ymax;aux++){
                           C.add(xmin,aux,1);
                           C.add(xmax,aux,1);
                    }
                    
        }
        friend class Matrix;
        friend void ponto2celula(const  ponto *actual,celula_robo *cell);
};

class mapa{
private:
    vector<objecto> m_Elements;
public:
    mapa(){}

  
    void Push(objecto aux) {
        m_Elements.push_back(aux);
    }
    void desenha(const  Matrix &T,Matrix &C){
        int i, total = m_Elements.size();
        for(i = 0 ; i < total; i++  ){
            m_Elements[i].add_object(T,C);
        }     
    }
     friend class Matrix;
};
//------------------------------INT MAIN-----------------------------------

int main() {
    
    T.add(1,1,0);T.add(1,2,-1);T.add(1,3,80);//
    T.add(2,1,1);T.add(2,2,0);T.add(2,3,1);// 
    T.add(3,1,0);T.add(3,2,0);T.add(3,3,1);//


    pc.baud(115200);
    float  x_desejado_ULTIMATE=90.0f;
    float  y_desejado_ULTIMATE=90.0f;
    ponto desejado_ULTIMATE = {.x_coord= x_desejado_ULTIMATE , .y_coord = y_desejado_ULTIMATE , .theta_coord =0.0f };
     
    
    float  d_desejado = 0.0f; //centimetros !!!
    //cria_parede(C);
    int j;
    for (j=1;j<81;j++){
        C.add(j,1,1);
        C.add(j,80,1);
        C.add(1,j,1);
        C.add(80,j,1);
    }
    //objecto obstaculo1( 20.0f,0.0f,10.0f , 10.0f);//obstaculo1.add_object(T,C);
    //objecto obstaculo2( 60.0f,60.0f,10.0f , 10.0f);//obstaculo2.add_object(T,C);
    //objecto obstaculo3( 5.0f,5.0f,35.0f ,  35.0f);//obstaculo3.add_object(T,C);
    //C.print();//while(1);
    objecto obstaculo1(50.0f,50.0f,20.0f ,20.0f);
    obstaculo1.add_object(T,C);
   // objecto obstaculo2( 50.0f,30.0f,100.0f , 15.0f);
    //obstaculo2.add_object(T,C);

    while(start_button==1){
            setSpeeds(0,0);
    }
    algo(&desejado_ULTIMATE,d_desejado);
    //printf("\nStop");
    setSpeeds(0,0);
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
        if(celula_actual.x==celula_desejada.x){
            if(celula_actual.y==celula_desejada.y){
                setSpeeds(0,0);
                }
                }
        t.start();
        getCountsAndReset(); // Call getCountsAndReset() to read the values of encoders
 
        coordenadas_novas(&ponto_actual,&ponto_anterior);
        ponto_anterior.theta_coord=ponto_actual.theta_coord;
        ponto_anterior.x_coord = ponto_actual.x_coord;
        ponto_anterior.y_coord = ponto_actual.y_coord;
        
        pc.printf("%f %f\n", ponto_actual.x_coord,ponto_actual.y_coord);
        
        ponto2celula(&ponto_actual,&celula_actual);
        
      //  pc.printf("celula actual  x = %f,   e y =  %f \n", celula_actual.x,celula_actual.y);
       // pc.printf("celula desej  x = %f,   e y =  %f \n", celula_desejada.x,celula_desejada.y);
        

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

        /*if(omega_E>185&&omega_R<omega_E){
            omega_R=135*omega_R/omega_E;
            omega_E=135;
        }
        else if(omega_R>185&&omega_E<omega_R){
            omega_E=135*omega_E/omega_R;
            omega_R=135;
        }
        else if(omega_E<25||omega_R<25){
            omega_E=omega_E*4;
            omega_R=omega_R*4;
            }*/
       // printf("\nV   %f,     %f\n", omega_E,omega_R);
        
       /* if (time_sum<time_control){
            setSpeeds((time_sum/time_control)*omega_E,(time_sum/time_control)*omega_R);    
        }
        else*/
            setSpeeds(omega_E,omega_R);
            
       // time_sum+=sample_time;
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
    erro_y = y_desejado - y_medido;  
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
    float i = 0.0f;
    float j = 0.0f;
    float check=0.0f;
    float Fr_x = 0.0f, Fr_y = 0.0f,Fr=0.0f;
    float Fx = 0.0f,  Fy = 0.0f;
    float Fa_x = 0.0f,  Fa_y = 0.0f;
    float  dist ,dt; 
    
    
    for (i = (cell_robot->x - lim_cell); i <= (cell_robot->x + lim_cell) ; i++){
         for (j = cell_robot->y - lim_cell; j <= cell_robot->y + lim_cell ; j++){
            if(i < 1 || i > 80 || j < 1 || j > 80){
                continue;
            }
            else{
                check=C.getNumber(i,j);
                if (check == 1.0f){
                    dist=sqrt(pow((cell_robot->x-i),2)+pow((cell_robot->y-j),2));
      
                    Fr=Fcr*check/(pow(dist,2));
                    Fx=Fr*(cell_robot->x-i)/dist;
                    Fy=Fr*(cell_robot->y-j)/dist;
                    Fr_x +=  Fx;
                    Fr_y += Fy; 
                    }
            }
        }
    }     

    dt = sqrt(pow(celula_desejada.x-cell_robot->x,2)+pow(celula_desejada.y-cell_robot->y,2));
    Fa_x = Fca *(celula_desejada.x-cell_robot->x)/dt;
    Fa_y = Fca*(celula_desejada.y-cell_robot->y)/dt;
   
    resultante->componente_x  =   Fa_x +  Fr_x;
    resultante->componente_y  =   Fa_y +  Fr_y;
    
    F_aux=resultante->componente_x;
    resultante->componente_x=resultante->componente_y;
    resultante->componente_y=-F_aux;
//    pc.printf("\n Forcas T : x %f y %f ",resultante->componente_x,resultante->componente_y);
    
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

