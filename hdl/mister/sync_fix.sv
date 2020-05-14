module sync_fix
(
    input clk,
    
    input sync_in,
    output sync_out
);

reg pol;
assign sync_out = sync_in ^ pol;

always @(posedge clk) begin
    static integer pos = 0, neg = 0, cnt = 0;
    reg s1,s2;

    s1 <= sync_in;
    s2 <= s1;

    if(~s2 & s1) neg <= cnt;
    if(s2 & ~s1) pos <= cnt;

    cnt <= cnt + 1;
    if(s2 != s1) cnt <= 0;

    pol <= pos > neg;
end

endmodule