high = 0b11111111;
mid = 0b11111111;
low = 0b11111111;


//MASKS

high_last = 0b00000010;
mid_first = 0b10000000;
mid_tl = 0b01110000;
mid_last = 0b00000010;

mid_low = 0b00000111;
mid_jump = 0b00001000;

green_tl = 0b00110000;
yellow_tl = 0b01010000;
red_tl = 0b00010000;

red_tick = 0;

while (1)
{

	if (GREEN)
	{
		red_tick_mask = 0;
		red_tck = 0;
		low = low>>1;					//move low over
		low |= (mid & mid_last);		//add the first bit in

		
		mid_temp = (mid & mid high);
		mid = mid >> 1;					//move mid over
		mid &= mid_low;				    //clear the top 5 bits of mid
		mid |= (((high & high_last) << 6) | (green_tl) | (mid_temp >> 4));
		
		high = high >> 1;
		if (!add_car) high |= high_first;
	}
		

	if (YELLOW)
	{

		low = low>>1;					//move low over
		low |= (mid & mid_last);		//add the first bit in

		
		mid_temp = (mid & mid high);
		mid = mid >> 1;					//move mid over
		mid &= mid_low;				    //clear the top 5 bits of mid
		mid |= (((high & high_last) << 6) | (yellow_tl) | (mid_temp >> 4));
		
		high = high >> 1;
		if (!add_car) high |= high_first;
	}
		
		
	if (RED)
	{

		low = low>>1;					//move low over
		low |= (mid & mid_last);		//add the first bit in

		
		mid = mid >> 1;					//move mid over
		mid &= mid_low;				    //clear the top 5 bits of mid
		
		mid |= (1<<4);					//light immediately after traffic light is off
		
		if (!red_tick) mid |= (((high & high_last) << 6)); //if we arent building up traffic move last bit of high over
		
		mid |= red_tl;									//update the traffic light colour
		
		if (!(mid & mid_first)) red_tick++;				//check the first bit to determine state of traffic buildup
														//if mid_7 is low then we need to build up traffic in the high byte 
		high = high >> 1;								//shift high over 1
		if (!add_car) high |= high_first;				//check the add_car condition
		
		red_tick_temp = red_tick;						//a place holder value for the logic performed on the value of red_tick
		
		while (red_tick_temp--)							//this while loop builds a mask that will clear some bits in high
		{												//to simulate traffic buildup
			red_tick_mask *= 2;
		}
		
		high &= (~(red_tick - 1) << 1);					//reset the bits in high byte that should have cars building
}
}