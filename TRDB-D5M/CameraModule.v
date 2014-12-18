
module CameraModule
(
		im_D5M_D,
		im_D5M_FVAL,
		im_D5M_LVAL,
		im_D5M_PIXLCLK,
		om_D5M_RESET_N,

		iom_D5M_STROBE,
		om_D5M_TRIGGER,
		im_DRY_RST_1,

		//Output Data
		om_rCCD_DATA,
		om_rCCD_LVAL,
		om_rCCD_FVAL
);
input		    [11:0]		im_D5M_D;
input		          		im_D5M_FVAL;
input		          		im_D5M_LVAL;
input		          		im_D5M_PIXLCLK;
output		          	om_D5M_RESET_N;

input		          		iom_D5M_STROBE;
output		          	om_D5M_TRIGGER;
input 						im_DRY_RST_1;

//Output Data
output reg		[11:0] 	om_rCCD_DATA;
output reg					om_rCCD_LVAL;
output reg					om_rCCD_FVAL;
assign	om_D5M_TRIGGER	=	1'b1;  // tRIGGER
assign	om_D5M_RESET_N	=	im_DRY_RST_1;

//D5M read 
always@(posedge im_D5M_PIXLCLK)
begin
	om_rCCD_DATA	<=	im_D5M_D;
	om_rCCD_LVAL	<=	im_D5M_LVAL;
	om_rCCD_FVAL	<=	im_D5M_FVAL;
end

endmodule