ISR(PCINT0_vect){
  if (PINB & B00001000){
    buttUpFlag = true;
  }

  if(PINB & B00000100){
    buttDownFlag = true;
  }

  if(PINB & B00000010){
    buttSelFlag = true;
  }

}
