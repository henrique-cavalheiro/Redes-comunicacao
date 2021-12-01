//Programa do controlador 3
//Controle da janela da estufa baseado na luminosidade

#define id 0b11
#include <Servo.h>

void faz_preambulo();
void detecta_preambulo();
void manda();
void recebe();
int checaParidade(unsigned long a);

bool flag_receb=0,flag_envio=0,flag_ini=0,flag_prea=0,flag_acabouTr=0,flag_entrada=0;
bool bit_anterior=0,bit_atual=0,atual_p=0,flag_CA=0,flag_CF=0;
int cont_receb=32,cont_envio=32,cont_prea=24;
unsigned long dado_env=0x0,dado_receb=0x0; 

//Servo myservo;

void setup(){
  TCCR1A = 0;              //confira timer para operação normal pinos OC1A e OC1B desconectados
  TCCR1B = 0;              //limpa registrador
    TCCR1B |= (1<<CS10);   //configura prescaler para 1: CS10 = 1
 
    TCNT1 = 0x7fff;          //65536-(16MHz/488Hz) = 32767 = 0x7FFF
                                       
    TIMSK1 &= !(1 << TOIE1);
  
  DDRB=0B00111101;
  
    //myservo.attach(7);
    //myservo.write(90);  
}

void loop() {
 
 unsigned short comando=0,leitura=0,porta=0,origem=0,sensor=0,recebidos=0;
 unsigned short comp_cabec=0,comp_data=0,tipo_serv=0,tam_carga=0,destino=0;
 bool paridade=0; 
  
  while(1){

    sensor=analogRead(A0);
    
    if(sensor < 768 && flag_CF==0){   //768 é o valor para 3/4 da resistência total
      PORTB |= 0B111100;        //enquanto a resistência for menor que 3/4
      dado_env=(0b10111010111101110010000000011000);
      paridade=checaParidade(dado_env);//do total, está dia, por isso abre a janela
      dado_env |= paridade;       //foram usados leds para simbolizar o servomotor
      flag_prea=1;            //porque a biblioteca servo.h utiliza o
      atual_p=0;            //mesmo timer para envio/recebimento de dados
      PORTB |= 0B1;     
      TIMSK1 |= (1 << TOIE1);
      flag_CF=1;
      flag_CA=0;
      }
    
    if(sensor >= 768 && flag_CA==0){
      PORTB &= 0B000011;    //se a resistência for maior ou igual a 3/4//está noite, então fecha a janela 
      dado_env=(0b10111010111101110010000000000110);//do total, está dia, por isso abre a janela
      paridade=checaParidade(dado_env);       //foram usados leds para simbolizar o servomotor
      dado_env |= paridade;             //porque a biblioteca servo.h utiliza o
      flag_prea=1;                  //mesmo timer para envio/recebimento de dados
      atual_p=0;
      PORTB |= 0B1;     
      TIMSK1 |= (1 << TOIE1);
      flag_CF=0;
      flag_CA=1;
      }           
    
  }
  
}

void faz_preambulo(){   //rotina que envia o preâmbulo

    if(cont_prea>1 && flag_prea==1){
      
        if (atual_p==1 && cont_prea!=2){
      PORTB = PORTB | atual_p;
        }
        if(atual_p==0 && cont_prea!=2){
      PORTB = PORTB & atual_p;
        }
        if(cont_prea==2){
      PORTB |= 0b1;
          flag_envio=1;
          cont_prea=24;
          flag_prea=0;
        }
        atual_p= !atual_p;
    cont_prea--;
  }     
}


void detecta_preambulo(){ //rotina que percebe o preâmbulo
  
  if(!flag_receb && !flag_envio){
    
  bit_anterior=bit_atual;
    if(0B1 & (PINB>>PINB1))
    bit_atual=1;
    else
        bit_atual=0;
    
    
  if(bit_anterior != bit_atual){
        flag_ini=1;
    }

  if(bit_atual==1 && bit_anterior==1 && flag_ini==1 && flag_receb==0){
        flag_receb=1;
    flag_ini=0;
        bit_anterior=0;
        bit_atual=0;
    }
  }
  
}

void manda(){

  bool bitMSB=0;
  
  if(cont_envio>0 && flag_envio==1){  //se tem dado para enviar, envia
    bitMSB=dado_env/2147483648;   //bitMSB é o bit mais significativo do dado
    if (bitMSB)
      PORTB = PORTB | 1;      //se ele for 1, seta a saída PB0 para 1
    else
      PORTB = PORTB & 0B11111110; //se ele for 0, coloca a saída PB0 para 0
        cont_envio--;
    dado_env=dado_env<<1;     //depois ele desloca o dado para a esquerda para que o MSB seja o próximo bit
    if(cont_envio==0){        //ele faz isso até o dado ser enviado completamente
            flag_acabouTr=1;
      flag_envio=0;       //quando envia ele reinicia as variáveis auxiliares e atualiza o armazenamento do buffer
            cont_envio=32;
    }
  }
}

void recebe(){

  if(cont_receb>0 && flag_receb==1){
      if(0B1 & (PINB>>PINB1)){    //se detectou o fim preâmbulo recebe
      dado_receb |= 0b1;      //coloca a entrada PB1 no LSB do buffer atual
        }
        else{
          dado_receb |= 0b0;
        }
        if(cont_receb>1)
      dado_receb=dado_receb<<1; //depois rotaciona para a esquerda para que o próximo LSB
        cont_receb--;         //seja o próximo valor de PB1, dessa forma o datagrama
    if(cont_receb==0){        //será formado bit a bit, começando pelo mais significativo
      flag_receb=0;       //quando termina, reinicia as variáveis auxiliares e atualiza
            cont_receb=32;        //o buffer
            flag_entrada=0;
            TIMSK1 &= !(1 << TOIE1);  
    } 
  }       
}

int checaParidade(unsigned long a) {
    while( a > 1 ) a = ( a >> 1 ) ^ ( a & 1 );
    return a;
}

ISR(TIMER1_OVF_vect)        //interrupção do TIMER1 
{
    TCNT1 = 0x7fff;       // Renicia TIMER
    
  if(flag_acabouTr){
    TIMSK1 &= !(1 << TOIE1);
    PORTB = PORTB & 0b11111110;
    flag_acabouTr=0;
  }
    
  if(flag_receb)
    recebe();
  
  if(!flag_receb && !flag_envio)
    detecta_preambulo();
    
  if(flag_envio)
      manda();
    
  if(flag_prea)
      faz_preambulo();
  
    
                              
}
