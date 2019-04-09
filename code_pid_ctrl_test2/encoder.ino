ISR(PCINT0_vect){
//  if(menu==1){
//    clk_State =   (PINB & B00000001); //pin 8 state? It is HIGH?
//    dt_State  =   (PINB & B00000010);
//    if (clk_State != Last_State){
//       // If the data state is different to the clock state, that means the encoder is rotating clockwise
//      if (dt_State != clk_State) {
//        set_temperature = set_temperature+0.5 ;
//      }else{
//        set_temperature = set_temperature-0.5;
//      }
//    }
//    Last_State = clk_State; // Updates the previous state of the clock with the current state
//  }
//
//  if(menu==2){
//    clk_State =   (PINB & B00000001); //pin 8 state?
//    dt_State  =   (PINB & B00000010);
//    if (clk_State != Last_State){
//      // If the data state is different to the clock state, that means the encoder is rotating clockwise
//      if (dt_State != clk_State) {
//        kp = kp+1 ;
//      }else{
//        kp = kp-1;
//      }
//    }
//    Last_State = clk_State; // Updates the previous state of the clock with the current state
//  }
//
//  if(menu==3){
//    clk_State =   (PINB & B00000001); //pin 8 state?
//    dt_State  =   (PINB & B00000010);
//    if (clk_State != Last_State){
//      // If the data state is different to the clock state, that means the encoder is rotating clockwise
//      if (dt_State != clk_State) {
//        ki = ki+1 ;
//      }else{
//        ki = ki-1;
//      }
//    }
//    Last_State = clk_State; // Updates the previous state of the clock with the current state
//  }
//
//  if(menu==4){
//    clk_State =   (PINB & B00000001); //pin 8 state?
//    dt_State  =   (PINB & B00000010);
//    if (clk_State != Last_State){
//      // If the data state is different to the clock state, that means the encoder is rotating clockwise
//      if (dt_State != clk_State) {
//        kd= kd+1 ;
//      }else{
//        kd = kd-1;
//      }
//    }
//    Last_State = clk_State; // Updates the previous state of the clock with the current state
//  }
//
  //Push button was pressed!
  if (PINC & B00000010){
    button_pressed = 1;
  }
  //We navigate through the 4 menus with each button pressed
  else if(button_pressed == 1){
    if(menu==4){
      menu = 0;
      PID_values_fixed=true;
      button_pressed=0;
      delay(1000);
    }
    if(menu==3){
      menu = menu + 1;
      button_pressed=0;
      kd = kd + 1;
      delay(1000);
    }
    if(menu==2){
      menu = menu + 1;
      button_pressed=0;
      ki = ki + 1;
      delay(1000);
    }
    if(menu==1){
      menu = menu + 1;
      button_pressed=0;
      kp = kp + 1;
      delay(1000);
    }
    if(menu==0 && PID_values_fixed != true){
      menu = menu + 1;
      button_pressed=0;
      set_temperature = set_temperature+1;
      delay(1000);
    }
    PID_values_fixed = false;
  }
}
