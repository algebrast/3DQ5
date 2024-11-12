/*
Copyright by Henry Ko and Nicola Nicolici
Department of Electrical and Computer Engineering
McMaster University
Ontario, Canada
*/

`timescale 1ns/100ps
`ifndef DISABLE_DEFAULT_NET
`default_nettype none
`endif

`include "define_state.h"
`define MATRIX_a 7'd76284
`define MATRIX_b 7'd104595
`define MATRIX_c 7'd25624
`define MATRIX_d 7'd53281
`define MATRIX_e 7'd132251

// This is the top module (same as experiment4 from lab 5 - just module renamed to "project")
// It connects the UART, SRAM and VGA together.
// It gives access to the SRAM for UART and VGA
module project (
		/////// board clocks                      ////////////
		input logic CLOCK_50_I,                   // 50 MHz clock

		/////// pushbuttons/switches              ////////////
		input logic[3:0] PUSH_BUTTON_N_I,         // pushbuttons
		input logic[17:0] SWITCH_I,               // toggle switches

		/////// 7 segment displays/LEDs           ////////////
		output logic[6:0] SEVEN_SEGMENT_N_O[7:0], // 8 seven segment displays
		output logic[8:0] LED_GREEN_O,            // 9 green LEDs

		/////// VGA interface                     ////////////
		output logic VGA_CLOCK_O,                 // VGA clock
		output logic VGA_HSYNC_O,                 // VGA H_SYNC
		output logic VGA_VSYNC_O,                 // VGA V_SYNC
		output logic VGA_BLANK_O,                 // VGA BLANK
		output logic VGA_SYNC_O,                  // VGA SYNC
		output logic[7:0] VGA_RED_O,              // VGA red
		output logic[7:0] VGA_GREEN_O,            // VGA green
		output logic[7:0] VGA_BLUE_O,             // VGA blue
		
		/////// SRAM Interface                    ////////////
		inout wire[15:0] SRAM_DATA_IO,            // SRAM data bus 16 bits
		output logic[19:0] SRAM_ADDRESS_O,        // SRAM address bus 18 bits
		output logic SRAM_UB_N_O,                 // SRAM high-byte data mask 
		output logic SRAM_LB_N_O,                 // SRAM low-byte data mask 
		output logic SRAM_WE_N_O,                 // SRAM write enable
		output logic SRAM_CE_N_O,                 // SRAM chip enable
		output logic SRAM_OE_N_O,                 // SRAM output logic enable
		
		/////// UART                              ////////////
		input logic UART_RX_I,                    // UART receive signal
		output logic UART_TX_O                    // UART transmit signal
);
	
logic resetn;

top_state_type top_state;

// For Push button
logic [3:0] PB_pushed;

// For VGA SRAM interface
logic VGA_enable;
logic [17:0] VGA_base_address;
logic [17:0] VGA_SRAM_address;

// For SRAM
logic [17:0] SRAM_address;
logic [15:0] SRAM_write_data;
logic SRAM_we_n;
logic [15:0] SRAM_read_data;
logic SRAM_ready;

//For upsampling and Conversion
logic [15:0] SRAM_Y;
logic [15:0] SRAM_U;
logic [15:0] SRAM_V;

logic [7:0] SRAM_Um5;
logic [7:0] SRAM_Um3;
logic [7:0] SRAM_Um1;
logic [7:0] SRAM_Up1;
logic [7:0] SRAM_Up3;
logic [7:0] SRAM_Up5;
logic [15:0] SRAM_U_prime;
logic [7:0] SRAM_U_prime_Buffer;


logic [7:0] SRAM_Vm5;
logic [7:0] SRAM_Vm3;
logic [7:0] SRAM_Vm1;
logic [7:0] SRAM_Vp1;
logic [7:0] SRAM_Vp3;
logic [7:0] SRAM_Vp5;
logic [15:0] SRAM_V_prime;
logic [7:0] SRAM_V_prime_Buffer;

logic [7:0] Temp1;
logic [7:0] Temp2;
logic [7:0] Temp3;
logic [7:0] Temp_aY;

logic [15:0] SRAM_RGB[2:0];

// For UART SRAM interface
logic UART_rx_enable;
logic UART_rx_initialize;
logic [17:0] UART_SRAM_address;
logic [15:0] UART_SRAM_write_data;
logic UART_SRAM_we_n;
logic [25:0] UART_timer;

logic [6:0] value_7_segment [7:0];

// For error detection in UART
logic Frame_error;

// For disabling UART transmit
assign UART_TX_O = 1'b1;

assign resetn = ~SWITCH_I[17] && SRAM_ready;

// Push Button unit
PB_controller PB_unit (
	.Clock_50(CLOCK_50_I),
	.Resetn(resetn),
	.PB_signal(PUSH_BUTTON_N_I),	
	.PB_pushed(PB_pushed)
);

VGA_SRAM_interface VGA_unit (
	.Clock(CLOCK_50_I),
	.Resetn(resetn),
	.VGA_enable(VGA_enable),
   
	// For accessing SRAM
	.SRAM_base_address(VGA_base_address),
	.SRAM_address(VGA_SRAM_address),
	.SRAM_read_data(SRAM_read_data),
   
	// To VGA pins
	.VGA_CLOCK_O(VGA_CLOCK_O),
	.VGA_HSYNC_O(VGA_HSYNC_O),
	.VGA_VSYNC_O(VGA_VSYNC_O),
	.VGA_BLANK_O(VGA_BLANK_O),
	.VGA_SYNC_O(VGA_SYNC_O),
	.VGA_RED_O(VGA_RED_O),
	.VGA_GREEN_O(VGA_GREEN_O),
	.VGA_BLUE_O(VGA_BLUE_O)
);

// UART SRAM interface
UART_SRAM_interface UART_unit(
	.Clock(CLOCK_50_I),
	.Resetn(resetn), 
   
	.UART_RX_I(UART_RX_I),
	.Initialize(UART_rx_initialize),
	.Enable(UART_rx_enable),
   
	// For accessing SRAM
	.SRAM_address(UART_SRAM_address),
	.SRAM_write_data(UART_SRAM_write_data),
	.SRAM_we_n(UART_SRAM_we_n),
	.Frame_error(Frame_error)
);

// SRAM unit
SRAM_controller SRAM_unit (
	.Clock_50(CLOCK_50_I),
	.Resetn(~SWITCH_I[17]),
	.SRAM_address(SRAM_address),
	.SRAM_write_data(SRAM_write_data),
	.SRAM_we_n(SRAM_we_n),
	.SRAM_read_data(SRAM_read_data),		
	.SRAM_ready(SRAM_ready),
		
	// To the SRAM pins
	.SRAM_DATA_IO(SRAM_DATA_IO),
	.SRAM_ADDRESS_O(SRAM_ADDRESS_O[17:0]),
	.SRAM_UB_N_O(SRAM_UB_N_O),
	.SRAM_LB_N_O(SRAM_LB_N_O),
	.SRAM_WE_N_O(SRAM_WE_N_O),
	.SRAM_CE_N_O(SRAM_CE_N_O),
	.SRAM_OE_N_O(SRAM_OE_N_O)
);

assign SRAM_ADDRESS_O[19:18] = 2'b00;

always @(posedge CLOCK_50_I or negedge resetn) begin
	if (~resetn) begin
		top_state <= S_IDLE;
		
		UART_rx_initialize <= 1'b0;
		UART_rx_enable <= 1'b0;
		UART_timer <= 26'd0;
		
		VGA_enable <= 1'b1;
	end else begin

		// By default the UART timer (used for timeout detection) is incremented
		// it will be synchronously reset to 0 under a few conditions (see below)
		UART_timer <= UART_timer + 26'd1;

		case (top_state)
		S_IDLE: begin
			VGA_enable <= 1'b1;  
			if (~UART_RX_I) begin
				// Start bit on the UART line is detected
				UART_rx_initialize <= 1'b1;
				UART_timer <= 26'd0;
				VGA_enable <= 1'b0;
				top_state <= S_UART_RX;
			end
		end

		S_UART_RX: begin
			// The two signals below (UART_rx_initialize/enable)
			// are used by the UART to SRAM interface for 
			// synchronization purposes (no need to change)
			UART_rx_initialize <= 1'b0;
			UART_rx_enable <= 1'b0;
			if (UART_rx_initialize == 1'b1) 
				UART_rx_enable <= 1'b1;

			// UART timer resets itself every time two bytes have been received
			// by the UART receiver and a write in the external SRAM can be done
			if (~UART_SRAM_we_n) 
				UART_timer <= 26'd0;

			// Timeout for 1 sec on UART (detect if file transmission is finished)
			if (UART_timer == 26'd49999999) begin
				top_state <= S_IDLE;
				UART_timer <= 26'd0;
			end
		end
		
		//Common case
		
		S_CONVERSION_DATA_0: begin
		
			if(/*cycle 1*/) begin
				//provide address for V
				SRAM_address <=   ;
				//shift U register
				SRAM_Um5<=SRAM_Um3;
				SRAM_Um3<=SRAM_Um1;
				SRAM_Um1<=SRAM_Up1;
				SRAM_Up1<=SRAM_Up3;
				SRAM_Up3<=SRAM_Up5;
				SRAM_Up5<=SRAM_U[7:0];
				//shift V register
				SRAM_Vm5<=SRAM_Vm3;
				SRAM_Vm3<=SRAM_Vm1;
				SRAM_Vm1<=SRAM_Vp1;
				SRAM_Vp1<=SRAM_Vp3;
				SRAM_Vp3<=SRAM_Vp5;
				SRAM_Vp5<=SRAM_V[7:0];
				
			end else begin
				//provide address for Bn,Rn+1
				SRAM_address <=   ;
				SRAM_write_data<=SRAM_RGB[1];	
				//shift U register
				SRAM_Um5<=SRAM_Um3;
				SRAM_Um3<=SRAM_Um1;
				SRAM_Um1<=SRAM_Up1;
				SRAM_Up1<=SRAM_Up3;
				SRAM_Up3<=SRAM_Up5;
				SRAM_Up5<=SRAM_U[15:8];
				//shift V register
				SRAM_Vm5<=SRAM_Vm3;
				SRAM_Vm3<=SRAM_Vm1;
				SRAM_Vm1<=SRAM_Vp1;
				SRAM_Vp1<=SRAM_Vp3;
				SRAM_Vp3<=SRAM_Vp5;
				SRAM_Vp5<=SRAM_V[15:8];

			end
		
			SRAM_U_prime_Buffer<=SRAM_Um1;
			SRAM_V_prime_Buffer<=SRAM_Vm1;
			Temp1<=MATRIX_a*SRAM_Y[15:8];
			
			SRAM_U_prime<=4'd159*(SRAM_Um1+SRAM_Up1);
			SRAM_V_prime<=4'd159*(SRAM_Vm1+SRAM_Vp1);
			
		   SRAM_RGB[2][15:8]<=Temp_aY-Temp1-Temp3;
			SRAM_RGB[2][7:0]<=Temp_aY+Temp2;
			state<=S_CONVERSION_DATA_1;

		end
		
		S_CONVERSION_DATA_1: begin
			if(/*cycle 1*/) begin
				//provide address for Y
				SRAM_address <=   ;
				
				//read U value 
				SRAM_U<=SRAM_read_data;
				//shift U register
				SRAM_Um5<=SRAM_Um1;
				SRAM_Um3<=SRAM_Um5;
				SRAM_Um1<=SRAM_Um3;
				SRAM_Up1<=SRAM_Up3;
				SRAM_Up3<=SRAM_Up5;
				SRAM_Up5<=SRAM_Up1;
				//shift V register
				SRAM_Vm5<=SRAM_Vm1;
				SRAM_Vm3<=SRAM_Vm5;
				SRAM_Vm1<=SRAM_Vm3;
				SRAM_Vp1<=SRAM_Vp3;
				SRAM_Vp3<=SRAM_Vp5;
				SRAM_Vp5<=SRAM_Vp1;
				
			end else begin
				//provide address for Gn+1,Bn+1
				SRAM_address <=   ;
				SRAM_write_data<=SRAM_RGB[2];	
				//shift U register
				SRAM_Um5<=SRAM_Um1;
				SRAM_Um3<=SRAM_Um5;
				SRAM_Um1<=SRAM_Um3;
				SRAM_Up1<=SRAM_Up3;
				SRAM_Up3<=SRAM_Up5;
				SRAM_Up5<=SRAM_Up1;
				//shift V register
				SRAM_Vm5<=SRAM_Vm1;
				SRAM_Vm3<=SRAM_Vm5;
				SRAM_Vm1<=SRAM_Vm3;
				SRAM_Vp1<=SRAM_Vp3;
				SRAM_Vp3<=SRAM_Vp5;
				SRAM_Vp5<=SRAM_Vp1;

			end

			Temp_aY<=Temp1;
			Temp1<=MATRIX_b*SRAM_V_prime_Buffer;
			SRAM_U_prime<=SRAM_U_prime-(4'd52*(SRAM_Um1+SRAM_Up1));
			SRAM_V_prime<=SRAM_V_prime-(4'd52*(SRAM_Vm1+SRAM_Vp1));
		
			state<=S_CONVERSION_DATA_2;
		end
		
		S_CONVERSION_DATA_2: begin
			if(/*cycle 1*/) begin
				//provide address for Rn,Gn
				SRAM_address <=   ;
				//write Rn,Gn;
				SRAM_write_data<=SRAM_RGB[0]
				
				//read V value 
				SRAM_V<=SRAM_read_data;
				//shift U register
				SRAM_Um5<=SRAM_Um1;
				SRAM_Um3<=SRAM_Um5;
				SRAM_Um1<=SRAM_Um3;
				SRAM_Up1<=SRAM_Up3;
				SRAM_Up3<=SRAM_Up5;
				SRAM_Up5<=SRAM_Up1;
				//shift V register
				SRAM_Vm5<=SRAM_Vm1;
				SRAM_Vm3<=SRAM_Vm5;
				SRAM_Vm1<=SRAM_Vm3;
				SRAM_Vp1<=SRAM_Vp3;
				SRAM_Vp3<=SRAM_Vp5;
				SRAM_Vp5<=SRAM_Vp1;
				
			end else begin
				//provide address for Y
				SRAM_address <=   ;
			
				//shift U register
				SRAM_Um5<=SRAM_Um1;
				SRAM_Um3<=SRAM_Um5;
				SRAM_Um1<=SRAM_Um3;
				SRAM_Up1<=SRAM_Up3;
				SRAM_Up3<=SRAM_Up5;
				SRAM_Up5<=SRAM_Up1;
				//shift V register
				SRAM_Vm5<=SRAM_Vm1;
				SRAM_Vm3<=SRAM_Vm5;
				SRAM_Vm1<=SRAM_Vm3;
				SRAM_Vp1<=SRAM_Vp3;
				SRAM_Vp3<=SRAM_Vp5;
				SRAM_Vp5<=SRAM_Vp1;

			end

			SRAM_RGB[0][15:8]<=Temp1+Temp_aY;
			Temp1<=MATRIX_c*SRAM_U_prime_Buffer;
			SRAM_U_prime<=SRAM_U_prime+(4'd21*(SRAM_Um1+SRAM_Up1));
			SRAM_V_prime<=SRAM_V_prime+(4'd21*(SRAM_Vm1+SRAM_Vp1));
			state<=S_CONVERSION_DATA_3;
		end

		
		
		S_CONVERSION_DATA_3: begin
			if(/*cycle 1*/) begin
				//provide address for Gn,Rn+1
				SRAM_address <=   ;
				//write Gn,Rn+1
				SRAM_write_data<=SRAM_RGB[1]
				
				//read Y value 
				SRAM_Y<=SRAM_read_data;
				//shift U register
				SRAM_Um5<=SRAM_Um1;
				SRAM_Um3<=SRAM_Um5;
				SRAM_Um1<=SRAM_Um3;
				SRAM_Up1<=SRAM_Up3;
				SRAM_Up3<=SRAM_Up5;
				SRAM_Up5<=SRAM_Up1;
				//shift V register
				SRAM_Vm5<=SRAM_Vm1;
				SRAM_Vm3<=SRAM_Vm5;
				SRAM_Vm1<=SRAM_Vm3;
				SRAM_Vp1<=SRAM_Vp3;
				SRAM_Vp3<=SRAM_Vp5;
				SRAM_Vp5<=SRAM_Vp1;
				
			end else begin
			
				//shift U register
				SRAM_Um5<=SRAM_Um1;
				SRAM_Um3<=SRAM_Um5;
				SRAM_Um1<=SRAM_Um3;
				SRAM_Up1<=SRAM_Up3;
				SRAM_Up3<=SRAM_Up5;
				SRAM_Up5<=SRAM_Up1;
				//shift V register
				SRAM_Vm5<=SRAM_Vm1;
				SRAM_Vm3<=SRAM_Vm5;
				SRAM_Vm1<=SRAM_Vm3;
				SRAM_Vp1<=SRAM_Vp3;
				SRAM_Vp3<=SRAM_Vp5;
				SRAM_Vp5<=SRAM_Vp1;

			end

			
			SRAM_RGB[0][7:0]<=(MATRIX_d*SRAM_V_prime_Buffer)+Temp_aY+Temp1;
			Temp2<=MATRIX_e*SRAM_U_prime_Buffer;
			Temp3<=MATRIX_a*SRAM_Y[7:0];
		
			state<=S_CONVERSION_DATA_4;
		end
		
		
		S_CONVERSION_DATA_4: begin
			if(/*cycle 1*/) begin
				//provide address for Gn+1,Bn+1
				SRAM_address <=   ;
				//write Gn,Rn+1
				SRAM_write_data<=SRAM_RGB[2]
				
				//read Y value 
				SRAM_Y<=SRAM_read_data;
			
			end else begin
			 //read new Y
				SRAM_Y<=SRAM_read_data;
			 
			end

			
			SRAM_RGB[1][15:8]<=Temp_aY+Temp2;
			Temp_aY<=Temp3;
			Temp1<=MATRIX_b*SRAM_U_prime;
		
			state<=S_CONVERSION_DATA_5;
		end
		
		
		
		S_CONVERSION_DATA_5: begin
			if(/*cycle 1*/) begin
				//provide address for Rn,Gn
				SRAM_address <=   ;
				//write Rn,Gn
				SRAM_write_data<=SRAM_RGB[0]
				
				//read Y value 
				SRAM_Y<=SRAM_read_data;
			
			end else begin
			 //read new Y
				SRAM_Y<=SRAM_read_data;
			 
			end

			
			SRAM_RGB[1][7:0]<=Temp_aY+Temp1;
			Temp1<=MATRIX_d*SRAM_V_prime;
			Temp2<=MATRIX_e*SRAM_U_prime;
			Temp3<=MATRIX_c*SRAM_U_prime;
		
			state<=S_CONVERSION_DATA_0;
		end	

		default: top_state <= S_IDLE;

		endcase
	end
end

// for this design we assume that the RGB data starts at location 0 in the external SRAM
// if the memory layout is different, this value should be adjusted 
// to match the starting address of the raw RGB data segment
assign VGA_base_address = 18'd0;

// Give access to SRAM for UART and VGA at appropriate time
assign SRAM_address = (top_state == S_UART_RX) ? UART_SRAM_address : VGA_SRAM_address;

assign SRAM_write_data = (top_state == S_UART_RX) ? UART_SRAM_write_data : 16'd0;

assign SRAM_we_n = (top_state == S_UART_RX) ? UART_SRAM_we_n : 1'b1;

// 7 segment displays
convert_hex_to_seven_segment unit7 (
	.hex_value(SRAM_read_data[15:12]), 
	.converted_value(value_7_segment[7])
);

convert_hex_to_seven_segment unit6 (
	.hex_value(SRAM_read_data[11:8]), 
	.converted_value(value_7_segment[6])
);

convert_hex_to_seven_segment unit5 (
	.hex_value(SRAM_read_data[7:4]), 
	.converted_value(value_7_segment[5])
);

convert_hex_to_seven_segment unit4 (
	.hex_value(SRAM_read_data[3:0]), 
	.converted_value(value_7_segment[4])
);

convert_hex_to_seven_segment unit3 (
	.hex_value({2'b00, SRAM_address[17:16]}), 
	.converted_value(value_7_segment[3])
);

convert_hex_to_seven_segment unit2 (
	.hex_value(SRAM_address[15:12]), 
	.converted_value(value_7_segment[2])
);

convert_hex_to_seven_segment unit1 (
	.hex_value(SRAM_address[11:8]), 
	.converted_value(value_7_segment[1])
);

convert_hex_to_seven_segment unit0 (
	.hex_value(SRAM_address[7:4]), 
	.converted_value(value_7_segment[0])
);

assign   
   SEVEN_SEGMENT_N_O[0] = value_7_segment[0],
   SEVEN_SEGMENT_N_O[1] = value_7_segment[1],
   SEVEN_SEGMENT_N_O[2] = value_7_segment[2],
   SEVEN_SEGMENT_N_O[3] = value_7_segment[3],
   SEVEN_SEGMENT_N_O[4] = value_7_segment[4],
   SEVEN_SEGMENT_N_O[5] = value_7_segment[5],
   SEVEN_SEGMENT_N_O[6] = value_7_segment[6],
   SEVEN_SEGMENT_N_O[7] = value_7_segment[7];

assign LED_GREEN_O = {resetn, VGA_enable, ~SRAM_we_n, Frame_error, UART_rx_initialize, PB_pushed};

endmodule
