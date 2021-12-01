//Programa do controlador 1
//Faz o controle da umidade da estufa

#define id 0b01

void faz_preambulo();
void detecta_preambulo();
void manda();
void recebe();
int checaParidade(unsigned long a);

bool flag_receb=0,flag_envio=0,flag_ini=0,flag_prea=1,flag_entrada=0,flag_acabouTr=0;
bool bit_anterior=0,bit_atual=0,atual_p=0,flag_mndSlc=0;
int cont_receb=32,cont_envio=32,cont_prea=24,recebidos=0;
unsigned long dado_env=0x0,dado_receb=0x00;


void setup() {
  
    TCCR1A = 0;                         //confira timer para operação normal pinos OC1A e OC1B desconectados
  TCCR1B = 0;                         //limpa registrador
    TCCR1B |= (1<<CS10);        //configura prescaler para 1: CS10 = 1
 
    TCNT1 = 0x7fff;                   //65536-(16MHz/488Hz) = 32767 = 0x0FFF
                                       
    TIMSK1 &= !(1 << TOIE1);

  DDRB=0B00000001;  //defina as entradas e saídas 
}


void loop() {
  
  unsigned short comando=0,leitura=0,porta=0,origem=0,sensor=0,tam_carga=0;
  unsigned short paridade=0,comp_cabec=0,comp_data=0,tipo_serv=0;
    bool flag_mdnSlc=0,flag_solic=0;
    
  while(1){
  
      sensor=analogRead(A0);    //leitura do sensor de umidade
    
      if(0B1 & (PINB>>PINB1)&& flag_entrada==0){
        flag_receb=0;     //rotina para reconhecer quando uma informação chega
        flag_entrada=1;     //e para sincronizar os controladores
            while(0B1 & (PINB>>PINB1));
          TCNT1 = 0xffff;
          TIMSK1 |= (1 << TOIE1);
      }
        
        if(recebidos>0){
            origem =(dado_receb/268435456) & (0b11);      //vê qual dos controladores enviou a informação
            tipo_serv=(dado_receb/67108864) & (0b11);     //pega o tipo de serviço
            comp_cabec =(dado_receb/4194304) & (0b1111);    //pega o comprimento do cabeçalho do datagrama
            comp_data =(dado_receb/131072) & (0b11111);     //pega o comprimento do datagrama
            porta=(dado_receb/32768) & (0b11);          //0-comando   1-leitura
            tam_carga=(dado_receb/2048) & (0b11);       //pega o tamanho da carga útil

            if(checaParidade(dado_env)==0){           //verifica se a paridade está correta

                if((porta == 0b00 || porta == 0b10) && tipo_serv == 0b10)   //destinado para processo de comando
                    comando = ((dado_receb>>1) & 0b1111);           //extrai somente os 4 bits do comando


                if((porta== 0b11 || porta == 0b01) && tipo_serv == 0b01)    //destinado para processo de leitura
                    leitura = ((dado_receb>>1) & 0X3FF);            //extrai os 10 bits de leitura 

                if(tipo_serv==0b11 && tam_carga==0)       //quando for uma solicitação, seta o flag de solicitação
                    flag_solic=1;
            }
            else{       //caso o dado esteja com problema na paridade, descarta o mesmo
                dado_env=0;
                recebidos=0;
            }
  }

        if(sensor <512){      //512 é um valor arbitrário que indica metade
            if(flag_mndSlc==0 && flag_acabouTr==0){ //da medida do sensor de umidade
            dado_env=(0b100111101111011100000 << 11) ;
            paridade=checaParidade(dado_env);   
            dado_env|=paridade;     //quando esta abaixo de 512, manda uma solicitação
            flag_prea=1;
            atual_p=0;
            flag_acabouTr=0;
            PORTB |= 0B1;     
            TIMSK1 |= (1 << TOIE1);
              flag_mndSlc=1;
            }
            if(recebidos>0 && leitura>=717){  //se a leitura recebida pós solicitação for
              PORTB |= 0b111100;        //dentro da faixa, liga a bomba
                recebidos=0;
            }
            if(recebidos>0 && leitura<717 && comando==0b0011){
              PORTB |= 0b111100;    //senão só liga quando o comando for o de ligar
              recebidos=0;
            }
        }
        else{       //quando a umidade for adequada novamente, o sistema pode
          flag_mdnSlc=0;  //mandar outra solicitação em outro momento
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


void detecta_preambulo(){   //rotina que percebe o preâmbulo
  
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
      dado_receb |= 0b1;        //coloca a entrada PB1 no LSB do buffer atual
          }
    else{
          dado_receb |= 0b0;
          }
        if(cont_receb>1)
      dado_receb=dado_receb<<1;         //depois rotaciona para a esquerda para que o próximo LSB           //seja o próximo valor de PB1, dessa forma o datagrama
        cont_receb--;
    if(cont_receb==0){        //será formado bit a bit, começando pelo mais significativo
      recebidos=1;        //quando termina, reinicia as variáveis auxiliares e atualiza
      flag_receb=0;       //o buffer
            cont_receb=32;
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
