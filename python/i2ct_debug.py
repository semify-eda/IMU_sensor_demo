
def debug_reg_printout(value):

    core_fsm        = value & 0x0000000f
    protocol_fsm    = (value & 0x000000f0) >> 4
    gif_wr_ctrl     = (value & 0x00000f00) >> 8
    gif_wr_update   = (value & 0x00001000) >> 12
    gif_rd_done     = (value & 0x00002000) >> 13
    gif_rd_update   = (value & 0x00004000) >> 14
    wb_ack          = (value & 0x00100000) >> 20
    wb_we           = (value & 0x00200000) >> 21
    wb_cyc          = (value & 0x00400000) >> 22
    wb_stb          = (value & 0x00800000) >> 23
    sda             = (value & 0x01000000) >> 24
    scl             = (value & 0x02000000) >> 25
    sda_o           = (value & 0x04000000) >> 26
    scl_o           = (value & 0x08000000) >> 27
    fsm_cnt         = (value & 0xf0000000) >> 28

    match(core_fsm):
        case 0: 
            core_state = "IDLE"        
        case 1: 
            core_state = "ADDR_REC_MSB"
        case 2: 
            core_state = "ADDR_REC"    
        case 3: 
            core_state = "RWN_REC"     
        case 4: 
            core_state = "ADDR_RWN_ACK"
        case 5: 
            core_state = "DATA_RX"     
        case 6: 
            core_state = "DATA_RX_ACK" 
        case 7: 
            core_state = "DATA_TX_LD"  
        case 8: 
            core_state = "DATA_TX"     
        case 9: 
            core_state = "DATA_TX_ACK" 

    match(protocol_fsm):
        case 0: 
            protocol_state = "IDLE"
        case 1: 
            protocol_state = "WR_ADDR_MSB"
        case 2: 
            protocol_state = "WR_ADDR_LSB"
        case 3: 
            protocol_state = "WR_DATA_REC"
        case 4: 
            protocol_state = "WR_BM"
        case 5: 
            protocol_state = "WR_BM_WAIT"
        case 6: 
            protocol_state = "RD_BM"
        case 7: 
            protocol_state = "RD_BM_WAIT"
        case 8: 
            protocol_state = "RD_DATA_TX"
        case 9: 
            protocol_state = "RD_DATA_TX_UPDATE"

    match(gif_wr_ctrl):
        case 0:
            gif_ctrl_state = "PROT_NONE"
        case 2:
            gif_ctrl_state = "PROT_START_WR"
        case 3:
            gif_ctrl_state = "PROT_START_RD"
        case 4:
            gif_ctrl_state = "PROT_DAT_WR"
        case 5:
            gif_ctrl_state = "PROT_DAT_RD"
        case 7:
            gif_ctrl_state = "PROT_STOP"

    print("I2CT Debug Entry (0x880F0): \n0x%x\n" % value)
    print(f"Core State: {core_fsm} -- {core_state}")
    print(f"Protocol Decoder State: {protocol_fsm} -- {protocol_state}")
    print(f"GIF Write Control: {gif_wr_ctrl} -- {gif_ctrl_state}")
    print(f"GIF Write Update: {gif_wr_update}")
    print(f"GIF Read Done: {gif_rd_done}")
    print(f"GIF Read Update: {gif_rd_update}")
    print(f"Wishbone ACK: {wb_ack}")
    print(f"Wishbone Write Enable: {wb_we}")
    print(f"Wishbone Cycle: {wb_cyc}")
    print(f"Wishbone Strobe: {wb_stb}")
    print(f"SDA In: {sda}")
    print(f"SDA Out: {sda_o}")
    print(f"SCL In: {scl}")
    print(f"SCL Out: {scl_o}") 
    print(f"FSM Count: {fsm_cnt}")