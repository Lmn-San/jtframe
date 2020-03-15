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
    Date: 14-3-2020 */

`timescale 1ns/1ps

// 8x8 tiles

module jtframe_credits #(
    parameter        PAGES  = 1,
                     COLW   = 4,       // bits per pixel colour component
    parameter [11:0] PAL0   = { 4'hf, 4'h0, 4'h0 },  // Red
    parameter [11:0] PAL1   = { 4'h0, 4'hf, 4'h0 },  // Green
    parameter [11:0] PAL2   = { 4'h3, 4'h3, 4'hf },  // Blue
    parameter [11:0] PAL3   = { 4'hf, 4'hf, 4'hf },  // White
    parameter        BLKPOL = 1'b1,
    parameter        SPEED  = 3
) (
    input               rst,
    input               clk,

    // input image
    input               pxl_cen,
    input               HB,
    input               VB,
    input [COLW*3-1:0]  rgb_in,
    input               enable,

    // output image
    output reg              HB_out,
    output reg              VB_out,
    output reg [COLW*3-1:0] rgb_out
);

localparam MSGW  = PAGES == 1 ? 10 :
                   (PAGES == 2 ? 11 :
                   (PAGES > 2 && PAGES <=4 ? 12 :
                   (PAGES > 5 && PAGES <=8 ? 13 : 14 ))); // Support for upto 16 pages
localparam MAXVISIBLE = PAGES*32*8;
localparam VPOSW = MSGW-2;

reg  [7:0] hpos;
reg  [VPOSW-1:0 ] vpos, vrender, vdump, vdump1;
(*keep*) wire [8:0]      scan_data;
(*keep*) wire [7:0]      font_data;
(*keep*) reg  [MSGW-1:0] scan_addr;
(*keep*) wire [9:0]      font_addr = {scan_data[6:0], vdump[2:0] };
wire visible = vrender < MAXVISIBLE;

jtframe_ram #(.dw(9), .aw(MSGW),.synfile("msg.hex")) u_msg(
    .clk    ( clk       ),
    .cen    ( 1'b1      ),
    .data   ( 9'd0      ),
    .addr   ( scan_addr ),
    .we     ( 1'b0      ),
    .q      ( scan_data )
);

jtframe_ram #(.aw(10),.synfile("font0.hex")) u_font(
    .clk    ( clk       ),
    .cen    ( 1'b1      ),
    .data   ( 8'd0      ),
    .addr   ( font_addr ),
    .we     ( 1'b0      ),
    .q      ( font_data )
);

reg  [1:0]      pal;
reg  [2:0]      pxl;

localparam SCROLL_EN = MSGW > 10;

// hb and vb are always active high
(*keep*) wire hb = BLKPOL ? HB : ~HB;
(*keep*) wire vb = BLKPOL ? VB : ~VB;
reg [7:0] pxl_data;

reg last_hb, last_vb;
reg [3:0] scr_base;

wire hb_edge = hb && !last_hb;

always @(posedge clk) begin
    if( rst ) begin
        hpos     <= 8'd0;
        vpos     <= {VPOSW{1'b0}};
        vrender  <= 8'd0;
        scr_base <= 4'd0;
    end else if(pxl_cen) begin
        last_hb <= hb;
        last_vb <= vb;
        if( hb_edge ) begin
            hpos    <= 8'd0;
            vrender <= vrender + 8'd1;
            vdump1  <= vpos + vrender;
            vdump   <= vdump1;
        end else if( !hb && hpos!=8'hff ) begin
            hpos <= hpos + 8'd1;
        end
        if ( vb ) begin
            vrender  <= 0;
            if( !last_vb && SCROLL_EN ) begin
                if( scr_base == SPEED ) begin
                    vpos <= vpos + 1;
                    scr_base <= 4'd0;
                end else scr_base <= scr_base + 4'd1;
            end
        end
        if( hpos[2:0]==3'd0 || hb || vb ) begin
            scan_addr <= { vdump[VPOSW-1:3], hpos[7:3] };
        end
        // Draw
        pxl <= { pal, pxl_data[7] };
        if( hpos[2:0]==3'd1) begin
            pal      <= scan_data[8:7];
            pxl_data <= font_data;
        end else
            pxl_data <= pxl_data << 1;
    end
end

//////////////////////////////////////////////////////////////////////////////
// Object Generator


`ifdef AVATARS
reg  [ 7:0] buf_din;
reg         buf_we0, buf_we1;
reg  [ 7:0] buf_addr0;

reg  [11:0] lut_addr;
wire [ 7:0] lut_data;

reg  [12:0] obj_addr;
wire [15:0] obj_data;
reg  [ 7:0] obj_pxl_data;

wire [ 7:0] pal_addr;
wire [11:0] pal_data;
reg  [11:0] obj_pxl;
reg  [ 7:0] obj_id, obj_x, obj_y;

reg         line = 1'b0;

jtframe_dual_ram #(.dw(8), .aw(9)) u_linebuffer(
    .clk0   ( clk       ),
    .clk1   ( clk       ),
    // Port 0: new data writes
    .data0  ( buf_din   ),
    .addr0  ( { line, buf_addr0 } ),
    .we0    ( buf_we0   ),
    .q0     (           ),
    // Port 1
    .data1  ( ~8'd0     ),
    .addr1  ( { ~line, hpos } ),
    .we1    ( buf_we1   ),
    .q1     ( pal_addr  )
);

jtframe_ram #(.dw(8), .aw(12),.synfile("lut.hex")) u_lut(
    .clk    ( clk       ),
    .cen    ( 1'b1      ),
    .data   ( 8'd0      ),
    .addr   ( lut_addr  ),
    .we     ( 1'b0      ),
    .q      ( lut_data  )
);

jtframe_ram #(.dw(16), .aw(13),.synfile("avatar.hex")) u_obj(
    .clk    ( clk       ),
    .cen    ( 1'b1      ),
    .data   ( 16'd0     ),
    .addr   ( obj_addr  ),
    .we     ( 1'b0      ),
    .q      ( obj_data  )
);

jtframe_ram #(.dw(12), .aw(4+4),.synfile("avatar_pal.hex")) u_pal(
    .clk    ( clk       ),
    .cen    ( 1'b1      ),
    .data   ( 12'd0     ),
    .addr   ( pal_addr  ),
    .we     ( 1'b0      ),
    .q      ( pal_data  )
);

wire [VPOSW-1:0] vsub = vrender - { lut_data, 3'b0 };
wire [3:0] obj_pal = 4'd0;

function [3:0] get_pxl;
    input [15:0] p;
    get_pxl = { p[12],p[8],p[4], p[0] };
endfunction

reg [4:0] st;
reg       first;

reg last_hb2; // this is one is not gated by pxl_cen

always @(posedge clk) begin
    last_hb2 <= hb;
    if( hb && !last_hb2 ) begin
        lut_addr <= 12'd0;
        first    <= 1'b1;
        st       <= 0;
        line     <= ~line;
        buf_we0  <= 1'b0;
        buf_addr0<= 8'd0;
    end else begin
        st <= st+1;
        case( st )
            0: begin
                if( lut_addr==12'd0 && !first ) st <= 0;
            end
            1: begin
                lut_addr <= lut_addr + 12'd1;
                first <= 1'b0;
            end
            2: begin
                lut_addr <= lut_addr + 12'd1;
                obj_id   <= lut_data;
            end
            3: begin
                lut_addr <= lut_addr + 12'd1;
                obj_x    <= lut_data;
            end
            4: begin
                lut_addr <= lut_addr + 12'd1;
                obj_y    <= lut_data;
                obj_addr <= { obj_id[6:0], vrender[3:0], 2'b0 }; // 7+4+2 = 13
                if( !({lut_data,3'b0} <= vrender && {lut_data,3'b0}+16  >= vrender) ) st <= 0;
                if( obj_id==8'hff ) begin // end of LUT
                    lut_addr <= 12'd0;
                    st <= 0;
                end
            end
            6: begin
                buf_addr0    <= { obj_x-8'd1, 3'b111 };
                obj_pxl_data <= obj_data;
            end
            7,8,9,10,    13,14,15,16,
            19,20,21,22, 25,26,27,28: begin
                buf_addr0    <= buf_addr0 + 1;
                buf_din      <= { obj_pal, get_pxl( obj_pxl_data) };
                buf_we0      <= 1'b1;
                obj_x        <= obj_x + 1;
                obj_pxl_data <= obj_pxl_data >> 1;
            end
            11: obj_addr[1:0] <= 2'b01;
            17: obj_addr[1:0] <= 2'b10;
            23: obj_addr[1:0] <= 2'b11;
            29: begin
                buf_we0<= 1'b0;
                st     <= 0;
            end
        endcase
    end
end

reg rd=1'b0;
reg [ 2:0] st2=3'd0;

always @(posedge clk ) begin
    st2 <= st2+1;
    case( st2 )
        3: begin
            obj_pxl <= pal_addr == 8'hff ? ~12'd0 : pal_data;
            buf_we1 <= 1'b1;
        end
        4: begin
            buf_we1 <= 1'b0;
            if( pxl_cen ) st2 <= 0; else st2 <= 4;
        end
    endcase
end

`else
wire [11:0] obj_pxl = ~12'h0;
`endif

/////////////////////////////////////////////////////////////////////////////
// Colour Mixer
// Merge the new image with the old
reg [COLW*3-1:0] old1, old2;
reg [3:0]        blanks;
wire [COLW*3-1:0] dim = { 1'b0, old2[R1:R0+1], 1'b0, old2[G1:G0+1], 1'b0, old2[B1:B0+1] };

localparam R1 = COLW*3-1;
localparam R0 = COLW*2;
localparam G1 = COLW*2-1;
localparam G0 = COLW;
localparam B1 = COLW-1;
localparam B0 = 0;

wire obj_visible = obj_pxl != ~12'd0;

always @(posedge clk) if(pxl_cen) begin
    old1 <= rgb_in;
    old2 <= old1;
    { blanks, HB_out, VB_out } <= { HB, VB, blanks };
    if( !enable )
        rgb_out <= old2;
    else begin
        if( (!pxl[0] && !obj_visible) || !visible 
            /*|| (blanks[1:0]^{2{~BLKPOL[0]}}!=2'b00)*/ ) begin
            rgb_out            <= dim;
        end else begin
            if( pxl[0] ) begin // CHAR
                case( pxl[2:1] )
                    2'd0: rgb_out <= PAL0;
                    2'd1: rgb_out <= PAL1;
                    2'd2: rgb_out <= PAL2;
                    2'd3: rgb_out <= PAL3;
                endcase
            end else rgb_out <= obj_pxl; // OBJ
        end
    end
end

endmodule