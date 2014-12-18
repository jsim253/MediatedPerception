

// synthesis VERILOG_INPUT_VERSION SYSTEMVERILOG_2005

module FrameBuffer
( 
	input 				imDRM5_CLOCK,
	input		  [11:0] imrCCD_DATA,
	input 	  			imrCCD_LVAL,
	input					imrCCD_FVAL,
	inout reg			oFlag,
	output reg [15:0] obuffer[799:0],
	output reg [15:0] oSbuffer[799:0]
);
	
/*Internal Data Lines*/	
reg locationX;
reg locationY;	
reg killSwitch;
	
//Parameters
parameter col = 800;
parameter row = 600;
	
initial begin
	oFlag = 0;
	locationX = 0;
	locationY = 0;
	killSwitch = 0;
end

always @ (posedge imDRM5_CLOCK) begin
	if(locationY == row) begin
		killSwitch = killSwitch + 1;
	end
	if(killSwitch == 0) begin
		// Find Proper Dimensions for the Image 
		obuffer[locationX] <= {imrCCD_DATA,imrCCD_LVAL,imrCCD_FVAL};
		locationX <= locationX + 1;
		if(locationX == col-1) begin
			locationX <= 0;
			oSbuffer <= obuffer;
			oFlag <= 1;
			locationY <= locationY + 1;
		end
	end
end


endmodule