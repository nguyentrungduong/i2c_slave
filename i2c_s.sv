////////////////////////////////////////////////////////////////////////////////
// Company     : 
//
// Filename    : i2c_s.sv
// Description : I2C slave
//
// Author      : Duong Nguyen
// Sogan       : "work hard in silence"
// History     : 
//
////////////////////////////////////////////////////////////////////////////////

module i2c_s
  #(parameter AW    = 16,
    parameter DW    = 32 )
  (
   input  logic             rstn           ,
   input  logic             clk            ,

   // I2C IO bufer interface
   input  logic             sda_i          ,
   output logic             sda_o          ,
   output logic             sda_en_o       ,
   input  logic             scl_i          ,
   output logic             scl_o          ,//don't use
   output logic             scl_en_o       ,//don't use


   //Internal interface 
   output logic             reg_wen_o      ,
   output logic [AW-1:0]    reg_addr_o     ,
   output logic [DW-1:0]    reg_wdata_o    ,
   output logic [DW/8-1:0]  reg_wstrb_o    , 
   output logic             reg_ren_o      , 
   input  logic [DW-1:0]    reg_rdata_i    ,
   input  logic             reg_rdata_vld_i
 
   );

//------------------------------------------------------------------------------
//Local parameter

localparam IDLE      = 4'd0;
localparam START     = 4'd1;
localparam DEV       = 4'd2;
localparam SACK      = 4'd3;
localparam WAIT      = 4'd4;
localparam ADDR      = 4'd5;
localparam WDATA     = 4'd6;
localparam SR        = 4'd7;
localparam RDATA     = 4'd8;
localparam MACK      = 4'd9;
localparam MNACK     = 4'd10;
localparam STOP      = 4'd11;




//------------------------------------------------------------------------------
//internal signal

logic            sda_sync;
logic            scl_sync;
logic            sda_q;
logic            scl_q;
logic            sda_rdet;
logic            sda_fdet;
logic            scl_rdet;
logic            scl_fdet;
logic            start_det;
logic            stop_det;

logic [3:0]      state;
logic [3:0]      nxt_state;

logic            st_idle;
logic            st_start;
logic            st_dev;
logic            st_sack;
logic            st_wait;
logic            st_addr;
logic            st_wdata;
logic            st_sr;
logic            st_rdata;
logic            st_mack;
logic            st_mnack;
logic            st_stop;


logic            idle_to_start;
logic            start_to_dev;
logic            dev_to_sack;
logic            dev_to_idle;
logic            sack_to_wait;
logic            sack_to_rdata;
logic            wait_to_addr;
logic            wait_to_wdata;
logic            wait_to_sr;
logic            wait_to_stop;
logic            addr_to_sack;
logic            wdata_to_sack;
logic            rdata_to_mack;
logic            rdata_to_mnack;
logic            mack_to_rdata;
logic            mack_to_mnack;
logic            mnack_to_stop;
logic            sr_to_dev;

logic [3:0]      nxt_lscl_cnt;
logic [3:0]      lscl_cnt;
logic            nxt_lscl_cnt_en;
logic            lscl_cnt_en;
logic            lscl_cnt_eq_8;

logic [3:0]      nxt_hscl_cnt;
logic [3:0]      hscl_cnt;
logic            nxt_hscl_cnt_en;
logic            hscl_cnt_en;
logic            hscl_cnt_eq_8;

logic [3:0]      nxt_bit_cnt;
logic [3:0]      bit_cnt;
logic            clr_bit_cnt;
logic            inc_bit_cnt;
logic            bit_cnt_eq_8;
logic            bit_wr;
logic            nxt_bit_wr;

logic [1:0]      addr_cnt;
logic [1:0]      nxt_addr_cnt;
logic            clr_addr_cnt;
logic            addr_cnt_eq_2;
logic            addr_cnt_eq_0;
logic            addr_cnt_eq_1;

logic [7:0]      nxt_dev_id;
logic [7:0]      dev_id;
logic            shift_dev_id_en;
logic            dev_id_match;

logic [15:0]     nxt_reg_addr;
logic [15:0]     reg_addr;
logic [15:0]     aligned_addr;
logic            shift_reg_addr_en;

logic [DW  :0]   nxt_wdata;
logic [DW  :0]   wdata;
logic            shift_wdata_en;
logic [2:0]      wcnt;
logic [2:0]      nxt_wcnt;
logic            clr_wcnt;
logic            wcnt_eq_0;
logic            wcnt_eq_1;
logic            wcnt_eq_2;
logic            wcnt_eq_3;
logic            wcnt_eq_4;

logic            w_1_byte_at_00;
logic            w_1_byte_at_01;
logic            w_1_byte_at_10;
logic            w_1_byte_at_11;
logic            w_4_byte_at_00;
logic [DW-1:0]   nxt_reg_wdata;

logic [3:0]      nxt_strb;
logic [3:0]      strb;

logic [DW-1:0]  nxt_rdata;
logic [DW-1:0]  rdata;
logic [DW-1:0]  rdata_t;
logic [DW-1:0]  rdata_00;
logic [DW-1:0]  rdata_01;
logic [DW-1:0]  rdata_10;
logic [DW-1:0]  rdata_11;
logic           shift_rdata_en;
logic           nxt_wen;

logic           nxt_sda_en;
logic           nxt_sda;
logic           nxt_ren;
logic           nxt_mack;
logic           mack;
logic           mack_cap_en;
logic           mack_cap_en_q;


//------------------------------------------------------------------------------
//SDA synchronizer


// 2FF synchronizer, synchronize gray write pointer to read frequency domain
i2c_2ff_sync #(1) sda_sync_00 (.clk(clk),.rstn(rstn),.din_meta_i(sda_i),.dout_sync_o(sda_sync));


//------------------------------------------------------------------------------
//SCL synchronizer


// 2FF synchronizer, synchronize gray write pointer to read frequency domain
i2c_2ff_sync #(1) scl_sync_00 (.clk(clk),.rstn(rstn),.din_meta_i(scl_i),.dout_sync_o(scl_sync));


//------------------------------------------------------------------------------
//Pipeline sda_sync and scl_sync one more flop

i2c_ffr #(1) ff_sda_i (.clk(clk),.rstn(rstn),.D(sda_sync),.Q(sda_q));

i2c_ffr #(1) ff_scl_i (.clk(clk),.rstn(rstn),.D(scl_sync),.Q(scl_q));

//------------------------------------------------------------------------------
// SDA rising edege detection

assign sda_rdet = (~sda_q) & sda_sync;

//------------------------------------------------------------------------------
// SDA falling edege detection

assign sda_fdet = (~sda_sync) & sda_q;

//------------------------------------------------------------------------------
// SCL rising edege detection

assign scl_rdet = (~scl_q) & scl_sync;


//------------------------------------------------------------------------------
// SCL falling edege detection

assign scl_fdet = (~scl_sync) & scl_q;

//------------------------------------------------------------------------------
// START condition detection

assign start_det = sda_fdet & scl_sync;

//------------------------------------------------------------------------------
// STOP condition detection

assign stop_det = sda_rdet & scl_sync;

//------------------------------------------------------------------------------
// I2C slave state machine

assign idle_to_start = st_idle  & start_det;

assign start_to_dev  = st_start & scl_fdet;

assign dev_to_sack   = st_dev   & bit_cnt_eq_8 & lscl_cnt_eq_8 & dev_id_match;
assign dev_to_idle   = st_dev   & bit_cnt_eq_8 & lscl_cnt_eq_8 & (~dev_id_match); 

assign sack_to_wait  = st_sack  & lscl_cnt_eq_8 & (~bit_wr);
assign sack_to_rdata = st_sack  & lscl_cnt_eq_8 & bit_wr; 
  
assign wait_to_addr  = st_wait & scl_fdet & (~addr_cnt_eq_2);
assign wait_to_wdata = st_wait & scl_fdet & addr_cnt_eq_2 & (~bit_wr) ;
assign wait_to_sr    = st_wait & start_det;
assign wait_to_stop  = st_wait & stop_det;

assign sr_to_dev     = st_sr & scl_fdet; 

assign addr_to_sack  = st_addr & lscl_cnt_eq_8 & bit_cnt_eq_8;

assign wdata_to_sack = st_wdata & bit_cnt_eq_8 & lscl_cnt_eq_8; 

assign rdata_to_mack = st_rdata & bit_cnt_eq_8 & lscl_cnt_eq_8; 

assign mack_to_mnack = st_mack & mack_cap_en_q & mack; 
assign mack_to_rdata = st_mack & lscl_cnt_eq_8;

assign mnack_to_stop = st_mnack & stop_det;


always_comb
  begin
  case (state)
    IDLE :
      begin
      nxt_state = idle_to_start ? START : IDLE; 
      end	
    START :
      begin
      nxt_state = start_to_dev ? DEV : START; 
      end	
    DEV :
      begin
      nxt_state = dev_to_sack ? SACK : 
                  dev_to_idle ? IDLE : DEV;
      end	
    SACK :
      begin
      nxt_state = sack_to_wait  ? WAIT  : 
                  sack_to_rdata ? RDATA : SACK;
      end	
    WAIT :
      begin
      nxt_state = wait_to_addr  ? ADDR  :
                  wait_to_wdata ? WDATA :
                  wait_to_stop  ? STOP  :
                  wait_to_sr    ? SR    : WAIT;  
      end	
    ADDR :
      begin
      nxt_state = addr_to_sack ? SACK : ADDR;
      end	
    WDATA :
      begin
      nxt_state = wdata_to_sack ? SACK : WDATA;
      end	
    SR :
      begin
      nxt_state = sr_to_dev ? DEV : SR ;
      end	
    RDATA :
      begin
      nxt_state = rdata_to_mack  ? MACK  : 
                  rdata_to_mnack ? MNACK : RDATA;
      end	
    MACK :
      begin
      nxt_state = mack_to_mnack ? MNACK :
                  mack_to_rdata ? RDATA : MACK;
      end	
    MNACK :
      begin
      nxt_state = mnack_to_stop ? STOP : MNACK;
      end	
    STOP :
      begin
      nxt_state = IDLE ;
      end	
    default :
      begin
      nxt_state = IDLE;
      end	
  endcase
  end


i2c_ffr #(4) ff_i2c_fsm (.clk(clk),.rstn(rstn),.D(nxt_state),.Q(state));


assign st_idle   = (state == IDLE);
assign st_start  = (state == START);
assign st_dev    = (state == DEV);
assign st_sack   = (state == SACK);
assign st_wait   = (state == WAIT);
assign st_addr   = (state == ADDR);
assign st_wdata  = (state == WDATA);
assign st_sr     = (state == SR);
assign st_rdata  = (state == RDATA);
assign st_mack   = (state == MACK);
assign st_mnack  = (state == MNACK);
assign st_stop   = (state == STOP);


//------------------------------------------------------------------------------
//high level SCL counter

//Enable operation of hscl counter
assign nxt_hscl_cnt_en = scl_rdet      ? 1'b1 :
                         hscl_cnt_eq_8 ? 1'b0 : hscl_cnt_en; 

i2c_ffr #(1) ff_hscl_cnt_en (.clk(clk),.rstn(rstn),.D(nxt_hscl_cnt_en),.Q(hscl_cnt_en));

//hscl_cnt 
assign nxt_hscl_cnt = hscl_cnt_en ? (hscl_cnt + 1'b1) : '0;   

i2c_ffr #(4) ff_hscl_cnt (.clk(clk),.rstn(rstn),.D(nxt_hscl_cnt),.Q(hscl_cnt));

assign hscl_cnt_eq_8 = (hscl_cnt == 4'd8);//--> event to capture write data


//------------------------------------------------------------------------------
//low level SCL counter

//Enable operation of lscl counter
assign nxt_lscl_cnt_en = scl_fdet      ? 1'b1 :
                         lscl_cnt_eq_8 ? 1'b0 : lscl_cnt_en; 


i2c_ffr #(1) ff_lscl_cnt_en (.clk(clk),.rstn(rstn),.D(nxt_lscl_cnt_en),.Q(lscl_cnt_en));

//lscl_cnt 
assign nxt_lscl_cnt = lscl_cnt_en ? (lscl_cnt + 1'b1) : '0;   

i2c_ffr #(4) ff_lscl_cnt (.clk(clk),.rstn(rstn),.D(nxt_lscl_cnt),.Q(lscl_cnt));

assign lscl_cnt_eq_8 = (lscl_cnt == 4'd8);//--> event to send SACK or read data


//------------------------------------------------------------------------------
//bit counter is use for counting 8 bit write data or sending 8 bit read data

assign clr_bit_cnt = st_idle | dev_to_sack | addr_to_sack | wdata_to_sack |
                     rdata_to_mack;

assign inc_bit_cnt = (st_dev  & hscl_cnt_eq_8)  |  //count 8 bit device id
                     wait_to_addr               |  //count first address bit.
                     (st_addr & hscl_cnt_eq_8)  |  //count next 7 bit address
                     wait_to_wdata              |  //count first wdata bit
                     (st_wdata & hscl_cnt_eq_8) |  //count next 7 bits wdata
                     sack_to_rdata              |  //count first sending rdata bit
                     (st_rdata & lscl_cnt_eq_8) ;  //count next 7 sending rdata bits 


assign nxt_bit_cnt = clr_bit_cnt ? '0 :
                     inc_bit_cnt ? (bit_cnt + 1'b1) : bit_cnt;

i2c_ffr #(4) ff_bit_cnt (.clk(clk),.rstn(rstn),.D(nxt_bit_cnt),.Q(bit_cnt));

assign bit_cnt_eq_8 = (bit_cnt == 4'd8); 


//------------------------------------------------------------------------------
//Latch WR bit, 0 : write , 1 : read

assign nxt_bit_wr = (st_dev & hscl_cnt_eq_8) ? sda_sync : bit_wr; 

i2c_ffr #(1) ff_bit_wr (.clk(clk),.rstn(rstn),.D(nxt_bit_wr),.Q(bit_wr));



//------------------------------------------------------------------------------
//Register address counter, slave must receive 2 bytes address

assign nxt_addr_cnt = st_idle      ? '0 :
                      addr_to_sack ? (addr_cnt + 1'b1) : addr_cnt; 

i2c_ffr #(2) ff_addr_cnt (.clk(clk),.rstn(rstn),.D(nxt_addr_cnt),.Q(addr_cnt));

assign addr_cnt_eq_0 = (addr_cnt == 2'd0);
assign addr_cnt_eq_1 = (addr_cnt == 2'd1);
assign addr_cnt_eq_2 = (addr_cnt == 2'd2);

//------------------------------------------------------------------------------
//capture and shift Device ID, Fixed Device ID[6:0] = 7'h18 (001_1000_x)

assign shift_dev_id_en = (st_dev & hscl_cnt_eq_8);

assign nxt_dev_id = st_idle         ? '0 :
                    shift_dev_id_en ? {dev_id[6:0],sda_sync} : dev_id;


i2c_ffr #(8) ff_dev_id (.clk(clk),.rstn(rstn),.D(nxt_dev_id),.Q(dev_id));

//checking device id is matched or not
assign dev_id_match = (dev_id[7:1] == 7'h18);

//------------------------------------------------------------------------------
//Shift Register address [15:0].

assign shift_reg_addr_en = (st_wait & hscl_cnt_eq_8 & addr_cnt_eq_0) |
                           (st_wait & hscl_cnt_eq_8 & addr_cnt_eq_1) |
                           (st_addr & hscl_cnt_eq_8);

assign nxt_reg_addr = st_idle           ? '0 :
                      shift_reg_addr_en ? {reg_addr[14:0],sda_sync} : reg_addr;

i2c_ffr #(16) ff_reg_addr (.clk(clk),.rstn(rstn),.D(nxt_reg_addr),.Q(reg_addr));



//generate aligned address
assign aligned_addr = {reg_addr[15:2],2'b00};

i2c_ffr #(16) ff_reg_aligned_addr (.clk(clk),.rstn(rstn),.D(aligned_addr),.Q(reg_addr_o));

//------------------------------------------------------------------------------
//Shift write data logic

assign shift_wdata_en = (st_wait & hscl_cnt_eq_8 & wcnt_eq_0 & addr_cnt_eq_2)| 
                        (st_wait & hscl_cnt_eq_8 & wcnt_eq_1 & addr_cnt_eq_2)| 
                        (st_wait & hscl_cnt_eq_8 & wcnt_eq_2 & addr_cnt_eq_2)| 
                        (st_wait & hscl_cnt_eq_8 & wcnt_eq_3 & addr_cnt_eq_2)| 
                        (st_wdata & hscl_cnt_eq_8); 

assign nxt_wdata = shift_wdata_en ? {wdata[31:0],sda_sync} : wdata; 

i2c_ffr #(DW+1) ff_wdata (.clk(clk),.rstn(rstn),.D(nxt_wdata),.Q(wdata));


//write data byte counter

assign clr_wcnt = st_idle | st_stop;

assign nxt_wcnt = clr_wcnt      ? '0 :
                  wdata_to_sack ? (wcnt + 1'b1) : wcnt;
                  

i2c_ffr #(3) ff_wcnt (.clk(clk),.rstn(rstn),.D(nxt_wcnt),.Q(wcnt));

assign wcnt_eq_0 = (wcnt == 3'd0);
assign wcnt_eq_1 = (wcnt == 3'd1);
assign wcnt_eq_2 = (wcnt == 3'd2);
assign wcnt_eq_3 = (wcnt == 3'd3);
assign wcnt_eq_4 = (wcnt == 3'd4);


//------------------------------------------------------------------------------
//Writing data to APB at STOP, checking WR bit, write 1 byte or 4 byte

assign w_1_byte_at_00 = (reg_addr[1:0] == 2'b00) & wcnt_eq_1 & st_stop & (~bit_wr);
assign w_1_byte_at_01 = (reg_addr[1:0] == 2'b01) & wcnt_eq_1 & st_stop & (~bit_wr);
assign w_1_byte_at_10 = (reg_addr[1:0] == 2'b10) & wcnt_eq_1 & st_stop & (~bit_wr);
assign w_1_byte_at_11 = (reg_addr[1:0] == 2'b11) & wcnt_eq_1 & st_stop & (~bit_wr);

assign w_4_byte_at_00 = (reg_addr[1:0] == 2'b00) & wcnt_eq_4 & st_stop;

assign nxt_reg_wdata = st_idle        ? '0 :
                       w_1_byte_at_00 ? {8'd0,8'd0,8'd0,wdata[8:1]} :
                       w_1_byte_at_01 ? {8'd0,8'd0,wdata[8:1],8'd0} :
                       w_1_byte_at_10 ? {8'd0,wdata[8:1],8'd0,8'd0} :
                       w_1_byte_at_11 ? {wdata[8:1],8'd0,8'd0,8'd0} :
                       w_4_byte_at_00 ? wdata[32:1]                 : '0;                       


i2c_ffr #(DW) ff_reg_wdata (.clk(clk),.rstn(rstn),.D(nxt_reg_wdata),.Q(reg_wdata_o));


//------------------------------------------------------------------------------
//Generate strobe, reg_wstrb_o

assign nxt_strb = st_idle        ? '0 :
                  w_1_byte_at_00 ? 4'b0001 :
                  w_1_byte_at_01 ? 4'b0010 :
                  w_1_byte_at_10 ? 4'b0100 :
                  w_1_byte_at_11 ? 4'b1000 :
                  w_4_byte_at_00 ? 4'b1111 : strb; //in case of read 1 byte,use strb to select                       


i2c_ffr #(4) ff_reg_strb (.clk(clk),.rstn(rstn),.D(nxt_strb),.Q(strb));

assign reg_wstrb_o = strb;

//------------------------------------------------------------------------------
//reg_wen_o

assign nxt_wen = st_stop & (~bit_wr);

i2c_ffr #(1) ff_reg_wen (.clk(clk),.rstn(rstn),.D(nxt_wen),.Q(reg_wen_o));


//------------------------------------------------------------------------------
//Latch 32 bit read data from APB

//capture read data and arrangement

assign rdata_00 = {reg_rdata_i[7:0]  ,reg_rdata_i[15:8] ,reg_rdata_i[23:16],reg_rdata_i[31:24]};
assign rdata_01 = {reg_rdata_i[15:8] ,reg_rdata_i[23:16],reg_rdata_i[31:24],reg_rdata_i[7:0]  };
assign rdata_10 = {reg_rdata_i[23:16],reg_rdata_i[31:24],reg_rdata_i[7:0]  ,reg_rdata_i[15:8] };
assign rdata_11 = {reg_rdata_i[31:24],reg_rdata_i[7:0]  ,reg_rdata_i[15:8] ,reg_rdata_i[23:16]};


assign rdata_t = (reg_rdata_vld_i & (reg_addr[1:0]==2'b00)) ? rdata_00 :   
                 (reg_rdata_vld_i & (reg_addr[1:0]==2'b01)) ? rdata_01 :   
                 (reg_rdata_vld_i & (reg_addr[1:0]==2'b10)) ? rdata_10 :   
                 (reg_rdata_vld_i & (reg_addr[1:0]==2'b11)) ? rdata_11 : '0;   

assign shift_rdata_en = sack_to_rdata | (st_rdata & lscl_cnt_eq_8);//checking shift read data later 

assign nxt_rdata = reg_rdata_vld_i ? rdata_t : 
                   shift_rdata_en  ? {rdata[30:0],1'b0} : rdata;


i2c_ffr #(DW) ff_reg_rdata (.clk(clk),.rstn(rstn),.D(nxt_rdata),.Q(rdata));

//------------------------------------------------------------------------------
//SDA out logic

assign nxt_sda_en = (st_sack | st_rdata) ? '1 : '0;

i2c_ffr #(1) ff_sda_en_o (.clk(clk),.rstn(rstn),.D(nxt_sda_en),.Q(sda_en_o));

assign nxt_sda = st_sack         ? '0 :
                 shift_rdata_en ? rdata[31] : sda_o;

i2c_ffr #(1) ff_sda_o (.clk(clk),.rstn(rstn),.D(nxt_sda),.Q(sda_o));


//------------------------------------------------------------------------------
//Reg read enable

assign nxt_ren = st_sack & bit_wr & scl_rdet;

i2c_ffr #(1) ff_reg_ren (.clk(clk),.rstn(rstn),.D(nxt_ren),.Q(reg_ren_o));

//------------------------------------------------------------------------------
//Capture MACK bit

assign mack_cap_en = st_sack & hscl_cnt_eq_8;
assign nxt_mack    = mack_cap_en ? sda_sync : mack; 

i2c_ffr #(1) ff_mack (.clk(clk),.rstn(rstn),.D(nxt_mack),.Q(mack));

i2c_ffr #(1) ff_mack_cap_en (.clk(clk),.rstn(rstn),.D(mack_cap_en),.Q(mack_cap_en_q));

endmodule 

