`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/15/2024 13:38:55 PM
// Design Name: Jia Yu Yang
// Module Name: fir
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,    
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);


// counter //
reg [3:0] count;

// counter for output number
reg [9:0] output_count;

// Store total length of data
reg [(pDATA_WIDTH-1):0] data_length;


// ap signals
reg ap_start;

//==============================================//
//       FSM for data transfer (stream)         //
//==============================================//
localparam IDLE = 2'd0;  // still reading coefficient
localparam LOAD = 2'd1;  // load in input (1 input for 1 cycle)
localparam MAC  = 2'd2;  // Computing (11 cycles)
localparam DONE  = 2'd3; // all done

reg [1:0] cur_state, next_state;

always@(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) cur_state <= IDLE;
    else cur_state <= next_state;
end

always@* begin
    case(cur_state)
        IDLE : next_state = (ap_start)? LOAD : IDLE;
        LOAD : next_state = MAC;
        MAC : begin
            if(count == Tape_Num) next_state = LOAD;
            else if(ss_tlast & output_count == data_length + 1'b1) next_state = DONE;
            else next_state = MAC;
        end 
        DONE : next_state = DONE;
    endcase
end

// counter and data length assignment
always@(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) data_length <= 'd0;
    else begin
        if(awaddr == 12'h10) data_length <= wdata;
        else data_length <= data_length;
    end
end


always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) count <= 'd0;
    else begin
        case (cur_state)
            LOAD : count <= 'd0;
            MAC : begin
                if(count < Tape_Num) count <= count + 1'b1;
                else count <= 'd0;
            end
            default count <= count;
        endcase
    end
end


//==============================================//
//              Axi_lite interface              //
//==============================================//

// address R/W ready handshake
reg awready_reg;
reg arready_reg;

assign awready = awready_reg;
assign arready = arready_reg;

// when address valid == 1, set address ready to 1 in next cycle
always@(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        awready_reg <= 'd0;
        arready_reg <= 'd0;
    end
    else begin
        awready_reg <= awvalid;
        arready_reg <= arvalid;
    end
end

// Return R / W ready hadnshake signal
assign rvalid = rready; 
assign wready = wvalid;


// ap signals
reg ap_start_flag;

reg ap_idle;
reg ap_done;
reg [(pDATA_WIDTH-1):0] ap_signal;

// ap_start: generate a one clock pulse when coefficient read is done
always@(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) ap_start_flag <= 1'b0;
    else begin
        if(awaddr == 12'h00) ap_start_flag <= 1'b1;
    end
end
always@(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) ap_start <= 1'b0;
    else begin
        if(awaddr == 12'h00 && ~ap_start_flag) ap_start <= 1'b1;
        else ap_start <= 1'b0;
    end
end


// ap_done: set to 1 if final output is generated
always@(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) ap_done <= 1'b0;
    else begin
        if (output_count == data_length & cur_state != IDLE) ap_done <= 1'b1;
        else ap_done <= ap_done;
    end
end

// ap_idle: reset when ap_start, set to 1 when final output is generated
always@(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) ap_idle <= 1'b1;
    else begin
        if (output_count == data_length) ap_idle <= 1'b1;
        else if (ap_start) ap_idle <= 1'b0;
        else ap_idle <= ap_idle;
    end
end

// concatenate all ap signals to ap_signals 
always@* begin
    ap_signal[31:0] = {29'b0, ap_idle, ap_done, ap_start};
end



// Control signals for BRAM

// BRAM enable 
reg tap_EN_reg;
assign tap_EN = tap_EN_reg;

// BRAM write enable 
reg [3:0]tap_WE_reg;
assign tap_WE = tap_WE_reg;

// BRAM write data
reg [(pDATA_WIDTH-1):0] tap_write;
assign tap_Di = tap_write;

// BRAM read data
reg [(pDATA_WIDTH-1):0] tap_read;

// if araddr is 12'h00, return ap signals (based on address map)
assign rdata = araddr == 12'h00 ? ap_signal : tap_read;

// BRAM address
reg [(pADDR_WIDTH-1):0] addr_reg;
assign tap_A = addr_reg;


// setting BRAM control signals
always@* begin
    // read
    if (rvalid) begin
        // excluding address 12'h00, since it is ap signal
        if(araddr != 12'h00) begin
            tap_read = tap_Do;
            tap_EN_reg = 1'b1;
            tap_WE_reg = 4'b0000;
            addr_reg = araddr-12'h20; 
            tap_write = 'd0;
        end
        // avoid latch !!!!!!!!!!
        else begin
            addr_reg = 'd0;
            tap_read  = 'd0;
            tap_EN_reg = 'd0;
            tap_WE_reg = 'd0;
            tap_write = 'd0;
        end
    end 
    // write
    else begin
        // excluding address 12'h00, since it is ap signal
        if(awaddr != 12'h00) begin
            addr_reg = awaddr-12'h20;
            tap_write = wdata;
            tap_EN_reg = 1'b1;
            tap_WE_reg = 4'b1111;
            tap_read  = 'd0;
        end
         // avoid latch !!!!!!!!!!
        else begin
            addr_reg = 'd0;
            tap_write = 'd0;
            tap_EN_reg = 'd0;
            tap_WE_reg = 'd0;
            tap_read  = 'd0;
        end
    end 
end 

// store coefficient to local registers when reading back to tb
reg [(pDATA_WIDTH-1):0] coef [Tape_Num-1 : 0];
integer i;
always@(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        for(i = 0; i < Tape_Num; i = i + 1)
            coef[i] <= 'd0;
    end
    else begin
        if(cur_state == IDLE) begin
            case (araddr)
                12'h20 : coef[0] <= tap_Do;
                12'h24 : coef[1] <= tap_Do;
                12'h28 : coef[2] <= tap_Do;
                12'h2c : coef[3] <= tap_Do;
                12'h30 : coef[4] <= tap_Do;
                12'h34 : coef[5] <= tap_Do;
                12'h38 : coef[6] <= tap_Do;
                12'h3c : coef[7] <= tap_Do;
                12'h40 : coef[8] <= tap_Do;
                12'h44 : coef[9] <= tap_Do;
                12'h48 : coef[10] <= tap_Do;
            endcase
        end
    end
end



//==============================================//
//               Stream interface               //
//==============================================//



// set ss_tready handshake signals (ready to receive data)
assign ss_tready = (ss_tvalid) & ((cur_state == LOAD) | ((cur_state == IDLE) & (next_state == LOAD)))? 1'b1 : 1'b0;

// Control signals for BRAM

// BRAM enable 
reg data_EN_reg;
assign data_EN = data_EN_reg;

// BRAM write
reg [3:0]data_WE_reg;
assign data_WE = data_WE_reg;

// BRAM write data
reg [(pDATA_WIDTH-1):0] data_write;
assign data_Di = data_write;

// BRAM address
reg [(pADDR_WIDTH-1):0] data_A_reg;
assign data_A = data_A_reg;

// setting data BRAM control signals
always@* begin
    if(ss_tready) begin
        data_A_reg = count << 2;
        data_EN_reg = 1'b1;
        data_WE_reg = 4'b1111;
        data_write = ss_tdata;
    end
    // avoid latch !!!!!!!!!!!
    else begin
        data_A_reg ='d0;
        data_EN_reg = 'd0;
        data_WE_reg = 'd0;
        data_write = 'd0;
    end
end


// one multiplier and one adder for fir //
wire [(pDATA_WIDTH-1):0] temp;
wire [(pDATA_WIDTH-1):0] cur_sum;
reg [(pDATA_WIDTH-1):0] prev_sum;
reg [(pDATA_WIDTH-1):0] cur_data;
reg [(pDATA_WIDTH-1):0] cur_coef;

assign temp = cur_data * cur_coef;
assign cur_sum = prev_sum + temp;


// shift register 
reg [(pDATA_WIDTH-1):0] shift [Tape_Num - 1 :0];

// when currrent state is LOAD, load in one input and shift all registers
always@(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        for(i = 0; i < Tape_Num; i = i + 1)
            shift[i] <= 'd0;
    end
    else begin
        if(cur_state == LOAD) begin
            for(i = 1 ; i < Tape_Num; i = i + 1) begin
                shift[i] <= shift[i-1];
            end
            shift[0] <= data_Do;
        end 
    end
end

// fir computing: one multiplication and one add in one cycle //
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        cur_data <= 'd0;
        prev_sum <= 'd0;
        cur_coef <= 'd0;
    end
    else begin
        case (cur_state)
            MAC : begin
                cur_data <= shift[count];
                prev_sum <= cur_sum;
                cur_coef <= coef[count];
            end
            default: begin
                cur_data <= 'd0;
                prev_sum <= 'd0;
                cur_coef <= 'd0;
            end
        endcase
    end
end


// set sm_tvalid handshake signals (valid for sending back outputdata, 11 cycles for one output)
assign sm_tvalid = ((cur_state == MAC) && (count == 'd11)) & sm_tready; 

// sm_tdata: output data to send back
reg [(pDATA_WIDTH-1):0] sm_tdata_reg;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) sm_tdata_reg <= 'd0;
    else sm_tdata_reg <= (sm_tvalid)? cur_sum : sm_tdata_reg;
end

assign sm_tdata = sm_tdata_reg;

// output counter
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) output_count <= 'd0;
    else begin
        if(sm_tvalid) begin
            output_count <= output_count + 1'b1;
        end
    end
    
end

// sm_tlast
assign sm_tlast = (cur_state == DONE);

endmodule
