module uart_receiver #(
    parameter longint signed CLOCK_HZ,
    parameter longint signed BAUD_RATE,
    parameter longint signed OVERSAMPLING_RATE,
    parameter longint signed DATA_BITS,
    parameter longint signed PARITY_MODE, // 0 = NONE, 1 = EVEN, 2 = ODD
    parameter longint signed STOP_BITS,

    localparam integer DATA_WIDTH = (DATA_BITS > 1) ? DATA_BITS : 1
) (
    input logic clk,
    input logic nRESET,
    input logic UART_RX,
    output logic UART_ERROR,
    output logic UART_DONE,
    output logic [DATA_WIDTH-1:0] UART_RX_DATA
);

    typedef enum logic [3:0] {
        IDLE,
        START_BIT_STAGE,
        DATA_BITS_STAGE,
        PARITY_BIT_STAGE,
        STOP_BITS_STAGE,
        DONE,
        ERROR
    } uart_receiver_state_t;

    uart_receiver_state_t state;
    uart_receiver_state_t next_state;

    localparam longint signed SAMPLE_RATE = BAUD_RATE*OVERSAMPLING_RATE;
    localparam integer MID_SAMPLE = (OVERSAMPLING_RATE + 1)/2;

    localparam integer ACCUMULATOR_WIDTH = (CLOCK_HZ+SAMPLE_RATE > 1) ? $clog2(CLOCK_HZ+SAMPLE_RATE) : 1;
    localparam integer OVERSAMPLING_COUNTER_WIDTH = (OVERSAMPLING_RATE > 1) ? $clog2(OVERSAMPLING_RATE) : 1;
    
    localparam integer DATA_COUNTER_WIDTH = (DATA_BITS > 1) ? $clog2(DATA_BITS) : 1;
    localparam integer STOP_COUNTER_WIDTH = (STOP_BITS > 1) ? $clog2(STOP_BITS) : 1;

    logic [STOP_COUNTER_WIDTH-1:0] stop_bit_counter;
    logic [ACCUMULATOR_WIDTH-1:0] phase_accumulator;
    logic sample_tick;
    logic [1:0] high_sample_count;
    logic [OVERSAMPLING_COUNTER_WIDTH-1:0] oversampling_counter;
    logic majority_result;
    logic [DATA_WIDTH-1:0] rx_data;
    logic [DATA_COUNTER_WIDTH-1:0] data_bit_counter;
    logic start_bit_valid;
    logic rx_meta;
    logic rx_sync;
    logic rx_sync_previous;
    logic start_edge;
    logic start_bit_checked;
    logic stop_bits_checked;
    logic parity_bit_checked;
    logic stop_bits_valid;
    logic parity_bit_valid;
    logic majority_ready;

    assign start_edge = rx_sync_previous && !rx_sync;


    initial begin
        if (OVERSAMPLING_RATE < 4) begin
            $fatal(1, "Oversampling rate must be at or above 4");
        end
        if (BAUD_RATE > (CLOCK_HZ/OVERSAMPLING_RATE)) begin
            $fatal(1, "Sample rate must be below hz of the input clock");
        end
        if (CLOCK_HZ < 1) begin
            $fatal(1, "Clock HZ must be at or above 1");
        end
        if (BAUD_RATE < 1) begin
            $fatal(1, "Baud rate must be at or above 1");
        end
        if (SAMPLE_RATE < 1) begin
            $fatal(1, "Sample rate turned out to be below 1");
        end
        if (DATA_BITS < 1) begin
            $fatal(1, "Data bits must be at or above 1");
        end
        if (STOP_BITS < 1) begin
            $fatal(1, "Stop bits must be at or above 1");
        end
        if (PARITY_MODE > 2 || PARITY_MODE < 0) begin
            $fatal(1, "Parity mode must be 0 for NONE, 1 for EVEN or 2 for ODD");
        end
        if (STOP_BITS > 2 || STOP_BITS < 1) begin
            $fatal(1, "Stop bits must be either 1 or 2");
        end
    end

    always_ff @(posedge clk or negedge nRESET) begin
        if (!nRESET) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin
        next_state = state;
        UART_ERROR = 1'd0;
        UART_DONE = 1'd0;
        case (state)
            IDLE: begin
                if (start_edge) begin
                    next_state = START_BIT_STAGE;
                end
            end
            START_BIT_STAGE: begin
                if (start_bit_valid && start_bit_checked) begin
                    next_state = DATA_BITS_STAGE;
                end else if (start_bit_checked && !start_bit_valid) begin
                    next_state = IDLE;
                end
            end
            DATA_BITS_STAGE: begin
                if (data_bit_counter + 1 == DATA_BITS && majority_ready) begin
                    if (PARITY_MODE == 0) begin
                        next_state = STOP_BITS_STAGE;
                    end else if (PARITY_MODE == 1 || PARITY_MODE == 2) begin
                        next_state = PARITY_BIT_STAGE;
                    end else begin
                        next_state = ERROR;
                    end
                end
                
            end
            PARITY_BIT_STAGE: begin
                if (parity_bit_valid && parity_bit_checked) begin
                    next_state = STOP_BITS_STAGE;
                end else if (parity_bit_checked && !parity_bit_valid) begin
                    next_state = ERROR;
                end
            end
            STOP_BITS_STAGE: begin
                if (stop_bits_valid && stop_bits_checked) begin
                    next_state = DONE;
                end else if (stop_bits_checked && !stop_bits_valid) begin
                    next_state = ERROR;
                end
            end
            DONE: begin
                next_state = IDLE;
                UART_DONE = 1'd1;
            end
            ERROR: begin
                next_state = IDLE;
                UART_ERROR = 1'd1;
            end
            default: begin
                next_state = IDLE;
            end
        endcase
    end

    always_ff @(posedge clk or negedge nRESET) begin
        if (!nRESET) begin
            rx_meta <= 1'd1;
            rx_sync <= 1'd1;
            rx_sync_previous <= 1'd1;
        end else begin
            rx_meta <= UART_RX;
            rx_sync <= rx_meta;
            rx_sync_previous <= rx_sync;
        end
    end

    always_ff @(posedge clk or negedge nRESET) begin
        if (!nRESET) begin
            start_bit_valid <= 1'd0;
        end else begin
            if (state == IDLE && next_state == START_BIT_STAGE) begin
                start_bit_valid <= 1'd0;
            end // implement start bit validation logic here
        end
    end

    always_ff @(posedge clk or negedge nRESET) begin
        if (!nRESET) begin
            start_bit_checked <= 1'd0;
            stop_bits_checked <= 1'd0;
            parity_bit_checked <= 1'd0;
        end else if (state == IDLE && start_edge) begin
            start_bit_checked <= 1'd0;
            stop_bits_checked <= 1'd0;
            parity_bit_checked <= 1'd0;
        end
    end

    always_ff @(posedge clk or negedge nRESET) begin
        if (!nRESET) begin
            phase_accumulator <= 1'd0;
            sample_tick <= 1'd0;
        end else begin
            sample_tick <= 1'd0;
            if (state == IDLE && start_edge) begin
                phase_accumulator <= 1'd0;
            end else if (state == START_BIT_STAGE || state == DATA_BITS_STAGE || state == PARITY_BIT_STAGE || state == STOP_BITS_STAGE) begin
                if (phase_accumulator + SAMPLE_RATE >= CLOCK_HZ) begin
                    sample_tick <= 1'd1;
                    phase_accumulator <= (phase_accumulator + SAMPLE_RATE) - CLOCK_HZ;
                end else begin
                    phase_accumulator <= phase_accumulator + SAMPLE_RATE;
                end
            end
        end
    end

    always_ff @(posedge clk or negedge nRESET) begin
        if (!nRESET) begin
            oversampling_counter <= 1'd0;
        end else if (state == IDLE && start_edge) begin
            oversampling_counter <= 1'd0;
        end else if (sample_tick && (state == START_BIT_STAGE || state == DATA_BITS_STAGE || state == PARITY_BIT_STAGE || state == STOP_BITS_STAGE)) begin
            if (oversampling_counter < (OVERSAMPLING_RATE-1)) begin
                oversampling_counter <= oversampling_counter + 1'd1;
            end else begin
                oversampling_counter <= 1'd0;
            end
        end
    end

endmodule