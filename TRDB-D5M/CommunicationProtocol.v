
// synthesis VERILOG_INPUT_VERSION SYSTEMVERILOG_2005
module CommunicationProtocol (
input imSysClock,
input imCLOCK,
inout iomDmar,
inout iomflag,
input [15:0] imBuffer[799:0],
output reg [15:0] omoutBuffer

);

parameter col = 800;
parameter row = 600;

reg [15:0] sBuffer[799:0];
reg completeFlag;
reg sendLocation;

initial begin
	completeFlag = 0;
	sendLocation = 0;
end

always@(posedge imSysClock) begin
	if(iomflag) begin
		sBuffer <= imBuffer;
		iomflag <= 0;
		completeFlag <= 1;
	end
end


always @(posedge imCLOCK) begin
	if(completeFlag == 1) begin
		iomDmar <= 0;
		omoutBuffer <= imBuffer[sendLocation];
		sendLocation <= sendLocation + 1;
		if(sendLocation == col-1) begin
			iomDmar <= 1;
			completeFlag <= 0;
		end
	end
end




endmodule