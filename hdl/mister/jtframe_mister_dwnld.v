/*  This file is part of JTFRAME.
    JTFRAME program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    JTFRAME program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with JTFRAME.  If not, see <http://www.gnu.org/licenses/>.

    Author: Jose Tejada Gomez. Twitter: @topapate
    Version: 1.0
    Date: 28-2-2021 */

module jtframe_mister_dwnld(
    input             rst,
    input             clk,

    output reg        downloading,

    input             hps_download, // signal indicating an active download
    input      [ 7:0] hps_index,        // menu index used to upload the file
    input             hps_wr,
    input      [26:0] hps_addr,         // in WIDE mode address will be incremented by 2
    input      [ 7:0] hps_dout,
    output            hps_wait,

    output            ioctl_rom_wr,
    output reg        ioctl_ram,
    output     [26:0] ioctl_addr,
    output     [ 7:0] ioctl_dout,

    // Configuration
    output reg [ 6:0] core_mod,
    input      [31:0] status,
    output     [31:0] dipsw

`ifdef JTFRAME_MR_DDRLOAD
    // DDR3 RAM
    ,output           ddram_clk,
    input             ddram_busy,
    output     [ 7:0] ddram_burstcnt,
    output     [28:0] ddram_addr,
    input      [63:0] ddram_dout,
    input             ddram_dout_ready,
    output            ddram_rd,
    output     [63:0] ddram_din,
    output     [ 7:0] ddram_be,
    output            ddram_we
`endif
);

localparam [7:0] IDX_ROM   = 8'h0,
                 IDX_MOD   = 8'h1,
                 IDX_NVRAM = 8'h2,
                 IDX_DIPSW = 8'd254;

assign hps_wait = 0;

always @(posedge clk) begin
    downloading <= hps_download && hps_index==IDX_ROM;
    ioctl_ram   <= hps_download && hps_index==IDX_NVRAM;
end

always @(posedge clk, posedge rst) begin
    if( rst ) begin
        core_mod <= 7'b01; // see readme file for documentation on each bit
    end else begin
        // The hps_addr[0]==1'b0 condition is needed in case JTFRAME_MR_FASTIO is enabled
        // as it always creates two write events and the second would delete the data of the first
        if (hps_wr && (hps_index==IDX_MOD) && hps_addr[0]==1'b0) core_mod <= hps_dout[6:0];
    end
end

`ifndef JTFRAME_MRA_DIP
    // DIP switches through regular OSD options
    assign dipsw        = status;
`else
    // Dip switches through MRA file
    // Support for 32 bits only for now.
    reg  [ 7:0] dsw[4];

    `ifndef SIMULATION
        assign dipsw = {dsw[3],dsw[2],dsw[1],dsw[0]};
    `else // SIMULATION:
        `ifndef JTFRAME_SIM_DIPS
            assign dipsw = ~32'd0;
        `else
            assign dipsw = `JTFRAME_SIM_DIPS;
        `endif
    `endif

    always @(posedge clk) begin
        if (hps_wr && (hps_index==IDX_DIPSW) && !hps_addr[24:2])
            dsw[hps_addr[1:0]] <= hps_dout;
    end
`endif

assign ioctl_rom_wr = hps_wr && (hps_index==IDX_ROM || hps_index==IDX_NVRAM);
assign ioctl_dout   = hps_dout;
assign ioctl_addr   = hps_addr;

/*
wire [63:0] dump_data;

jtframe_dual_ram #(.dw(64),.aw(5))) u_buffer(
    .clk0   ( clk   ),
    .clk1   ( clk   ),
    // Port 0: write
    .data0  ( ddram_din ),
    .addr0  ( ddram_cnt ),
    .we0    ( ddram_we  ),
    .q0     (           ),
    // Port 1: read
    .data1  (           ),
    .addr1  ( dump_cnt  ),
    .we1    ( 1'b0      ),
    .q1     ( dump_data )
);
*/
endmodule