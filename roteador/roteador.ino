//Programa do roteador/concentrador da rede

void faz_preambulo();
void detecta_preambulo();
void manda();
void recebe();

bool flag_receb=0,flag_envio=0,flag_ini=0,flag_prea=0,flag_entrada=0,flag_acabouTr=0;
bool bit_anterior=0,bit_atual=0,atual_p=0;
int cont_receb=32,cont_envio=32,cont_prea=24,desl_envio=0,desl_receb=0,recebidos=0;
unsigned long dado_env=0x0,dado_receb=0x00;



void setup(){
  TCCR1A = 0;                     //confira timer para operação normal pinos OC1A e OC1B desconectados
  TCCR1B = 0;                     //limpa registrador
    TCCR1B |= (1<<CS10);      //configura prescaler para 1: CS10 = 1
 
    TCNT1 = 0x7FFF;                 //65536-(16MHz/488Hz) = 32767 = 0x7FFF
                                       
    TIMSK1 &= !(1 << TOIE1);
  
  DDRB=0B00010101;        //define as entradas e saídas
    DDRD=0B00000000;        //a porta D é usada como auxiliar para ver os 4  primeiros bits
    PORTB &= 0B11101010;      //do dado recebido
  
}


void loop() {
  
  unsigned long buffer[10];   //buffer é o número de dados que pode ficar armazenados na memória
  int con_buff=0;
  int id;


  while(1){ 
    
    
    if((0B1 & (PINB>>PINB1)) && flag_entrada==0){ 
      dado_receb=0x00;    //quando percebe um nivel alto no pino de entrada
      flag_entrada=1;   //faz a sincronia e liga o timer, o qual será utilizado
      desl_receb=1;     //para receber os dados
      flag_receb=0;
      bit_anterior=0;
      bit_atual=0;
      while(0B1 & (PINB>>PINB1));
      TCNT1 = 0xffff;
      TIMSK1 |= (1 << TOIE1);
    }
    if((0B1 & (PINB>>PINB3)) && flag_entrada==0){
      dado_receb=0x00;    //mesma coisa que o anterior porém no pino
      flag_entrada=1;   //que serve como entrada de outro controlador
      desl_receb=3;
      flag_receb=0;
      bit_anterior=0;
      bit_atual=0;
      while(0B1 & (PINB>>PINB1));
      TCNT1 = 0xffff;
      TIMSK1 |= (1 << TOIE1);
    }

    if((0B1 & (PINB>>PINB5)) && flag_entrada==0){
      dado_receb=0x00;  
      flag_entrada=1;   //análogo aos dois anteriores
      desl_receb=5;
      flag_receb=0;
      bit_anterior=0;
      bit_atual=0;
      while(0B1 & (PINB>>PINB5));
      TCNT1 = 0xffff;
      TIMSK1 |= (1 << TOIE1);
    }

    if(recebidos>0 && flag_entrada==0){ //quando recebe um dado
      dado_env=dado_receb;
      id=(dado_receb/1073741824);
      switch (id) {     //baseado no seu id de destino, decide o pino de saída
        case 0b01:
          desl_envio=4;
          break;
        case 0b10:
          desl_envio=2;
          break;
        case 0b11:
          desl_envio=0;
          break;
        default:
        desl_envio=2;
        }
        flag_prea=1;
        flag_acabouTr=0;
        flag_entrada=1;   //e depois envia o dado recebido para o destino desejado
        PORTB |= (0B1 << desl_envio);
        TIMSK1 |= (1 << TOIE1);
    }
  }
  
}

void faz_preambulo(){   //rotina que envia o preâmbulo antes de enviar os dados
  
  
  if(cont_prea>1 && flag_prea==1){
    if (atual_p==1 && cont_prea!=2){
      PORTB = PORTB | (0b1 << desl_envio); 
    }
    if(atual_p==0 && cont_prea!=2){
      PORTB = PORTB & (~(0b1 << desl_envio));
    }
    if(cont_prea==2){
    PORTB |= (0b1 << desl_envio);
        flag_envio=1;
        cont_prea=24;
        flag_prea=0;
        atual_p=0;
    }
    atual_p= !atual_p;
    cont_prea--;
  }     
}


void detecta_preambulo(){ //rotina que reconhece o preâmbulo
  
  if(!flag_receb && !flag_envio){

  bit_anterior=bit_atual;

    if(0B1 & (PINB>>desl_receb)){
    bit_atual=1;
    }
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

  bool bitMSB;
  
  if(cont_envio>0 && flag_envio==1){  //se tem dado para enviar, envia
    bitMSB=dado_env/2147483648;   //bitMSB é o bit mais significativo do dado
    if (bitMSB)
      PORTB = PORTB | (0b1<<desl_envio);  //se ele for 1, seta a saída PB0 para 1
    else
      PORTB = PORTB & (~(0b1<<desl_envio)); //se ele for 0, coloca a saída PB0 para 0
    cont_envio--;
    dado_env=dado_env<<1;     //depois ele desloca o dado para a esquerda para que o MSB seja o próximo bit
    if(cont_envio==0){        //ele faz isso até o dado ser enviado completamente
          recebidos=0;
        flag_envio=0;       //quando envia ele reinicia as variáveis auxiliares e atualiza o armazenamento do buffer
          cont_envio=32;
          flag_entrada=0;
          flag_acabouTr=1;
            flag_prea=0;
    }
  }
}

void recebe(){

  if(cont_receb>0 && flag_receb==1){
      if(0B1 & (PINB>>desl_receb)){   //se detectou o fim preâmbulo recebe
    dado_receb |= 0b1;        //coloca a entrada PB1 no LSB do buffer atual
      }
      else{
        dado_receb |= 0b0;

      }
      if(cont_receb>1){
        dado_receb = dado_receb<<1; //depois rotaciona para a esquerda para que o próximo LSB 
      }
      cont_receb--;             //seja o próximo valor de PB1, dessa forma o datagrama
    if(cont_receb==0){      //será formado bit a bit, começando pelo mais significativo
      flag_receb=0;     //quando termina, reinicia as variáveis auxiliares
          cont_receb=32;
          recebidos=1;
          flag_entrada=0;
            PORTD = ((dado_receb)/268435456)<<2;
          TIMSK1 &= !(1 << TOIE1);
      } 
    }
        
}

ISR(TIMER1_OVF_vect)        //interrupção do TIMER1 
{
    TCNT1 = 0x7fff;       // Renicia TIMER
  
  if(flag_acabouTr){    //depois de enviar um dado, retorna a saída para nível baixo
    PORTB = PORTB & (~(0b1<<desl_envio));
    TIMSK1 &= !(1 << TOIE1);
    flag_acabouTr=0;
  }
    
  if(flag_receb)
    recebe();
  
  if(flag_receb==0 && flag_envio==0)
    detecta_preambulo();
    
  if(flag_envio)
      manda();
    
  if(flag_prea)
      faz_preambulo();                              
}
