module rgb2ypbpr (
	input [5:0]     red,
	input [5:0]     green,
	input [5:0]     blue,

	output [5:0]    y,
	output [5:0]    pb,
	output [5:0]    pr
);

`ifndef IVERILOG
wire [5:0] yuv_full[225] = '{
  6'd0,   6'd0,  6'd0,  6'd0,  6'd1,  6'd1,  6'd1,  6'd1,
  6'd2,   6'd2,  6'd2,  6'd3,  6'd3,  6'd3,  6'd3,  6'd4,
  6'd4,   6'd4,  6'd5,  6'd5,  6'd5,  6'd5,  6'd6,  6'd6,
  6'd6,   6'd7,  6'd7,  6'd7,  6'd7,  6'd8,  6'd8,  6'd8,
  6'd9,   6'd9,  6'd9,  6'd9, 6'd10, 6'd10, 6'd10, 6'd11,
  6'd11, 6'd11, 6'd11, 6'd12, 6'd12, 6'd12, 6'd13, 6'd13,
  6'd13, 6'd13, 6'd14, 6'd14, 6'd14, 6'd15, 6'd15, 6'd15,
  6'd15, 6'd16, 6'd16, 6'd16, 6'd17, 6'd17, 6'd17, 6'd17,
  6'd18, 6'd18, 6'd18, 6'd19, 6'd19, 6'd19, 6'd19, 6'd20,
  6'd20, 6'd20, 6'd21, 6'd21, 6'd21, 6'd21, 6'd22, 6'd22,
  6'd22, 6'd23, 6'd23, 6'd23, 6'd23, 6'd24, 6'd24, 6'd24,
  6'd25, 6'd25, 6'd25, 6'd25, 6'd26, 6'd26, 6'd26, 6'd27,
  6'd27, 6'd27, 6'd27, 6'd28, 6'd28, 6'd28, 6'd29, 6'd29,
  6'd29, 6'd29, 6'd30, 6'd30, 6'd30, 6'd31, 6'd31, 6'd31,
  6'd31, 6'd32, 6'd32, 6'd32, 6'd33, 6'd33, 6'd33, 6'd33,
  6'd34, 6'd34, 6'd34, 6'd35, 6'd35, 6'd35, 6'd35, 6'd36,
  6'd36, 6'd36, 6'd36, 6'd37, 6'd37, 6'd37, 6'd38, 6'd38,
  6'd38, 6'd38, 6'd39, 6'd39, 6'd39, 6'd40, 6'd40, 6'd40,
  6'd40, 6'd41, 6'd41, 6'd41, 6'd42, 6'd42, 6'd42, 6'd42,
  6'd43, 6'd43, 6'd43, 6'd44, 6'd44, 6'd44, 6'd44, 6'd45,
  6'd45, 6'd45, 6'd46, 6'd46, 6'd46, 6'd46, 6'd47, 6'd47,
  6'd47, 6'd48, 6'd48, 6'd48, 6'd48, 6'd49, 6'd49, 6'd49,
  6'd50, 6'd50, 6'd50, 6'd50, 6'd51, 6'd51, 6'd51, 6'd52,
  6'd52, 6'd52, 6'd52, 6'd53, 6'd53, 6'd53, 6'd54, 6'd54,
  6'd54, 6'd54, 6'd55, 6'd55, 6'd55, 6'd56, 6'd56, 6'd56,
  6'd56, 6'd57, 6'd57, 6'd57, 6'd58, 6'd58, 6'd58, 6'd58,
  6'd59, 6'd59, 6'd59, 6'd60, 6'd60, 6'd60, 6'd60, 6'd61,
  6'd61, 6'd61, 6'd62, 6'd62, 6'd62, 6'd62, 6'd63, 6'd63,
  6'd63
};
`else 
wire [5:0] yuv_full[225]; // Leave it empty. Output will be X's but
  // it won't stop iVerilog from running
`endif


wire [18:0]  y_8 = 19'd04096 + ({red, 8'd0} + {red, 3'd0}) + ({green, 9'd0} + {green, 2'd0}) + ({blue, 6'd0} + {blue, 5'd0} + {blue, 2'd0});
wire [18:0] pb_8 = 19'd32768 - ({red, 7'd0} + {red, 4'd0} + {red, 3'd0}) - ({green, 8'd0} + {green, 5'd0} + {green, 3'd0}) + ({blue, 8'd0} + {blue, 7'd0} + {blue, 6'd0});
wire [18:0] pr_8 = 19'd32768 + ({red, 8'd0} + {red, 7'd0} + {red, 6'd0}) - ({green, 8'd0} + {green, 6'd0} + {green, 5'd0} + {green, 4'd0} + {green, 3'd0}) - ({blue, 6'd0} + {blue , 3'd0});

wire [7:0] y_i  = ( y_8[17:8] < 16) ? 8'd16 : ( y_8[17:8] > 235) ? 8'd235 :  y_8[15:8];
wire [7:0] pb_i = (pb_8[17:8] < 16) ? 8'd16 : (pb_8[17:8] > 240) ? 8'd240 : pb_8[15:8];
wire [7:0] pr_i = (pr_8[17:8] < 16) ? 8'd16 : (pr_8[17:8] > 240) ? 8'd240 : pr_8[15:8];

assign pr = yuv_full[pr_i - 8'd16];
assign y  = yuv_full[y_i  - 8'd16];
assign pb = yuv_full[pb_i - 8'd16];

endmodule
