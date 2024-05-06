module spi ( // define SPI 
input clk, newd, rst, // newd = new data. The input can be "0" or "1" logi. 
input [11:0] din, // 12-bit wide bus
output reg sclk, cs, mosi // cs = what chip selected. the usage of clk is to initiate the system, and the usage of sclk is 
synchronizing the data transfer between the SPI master and slave devices.
    );
  
  typedef enum bit [1:0] {idle = 2'b00, enable = 2'b01, send = 2'b10, comp = 2'b11 } state_type; // current state of SPI module.
  state_type state = idle;
  
  int countc = 0; 

 always@(posedge clk) // Change of status every positive clock. 
  begin
    if(rst == 1'b1) begin
      countc <= 0; // used to initialize or reset the counter to zero before it starts counting.
      sclk <= 1'b0;
    end
    else begin 
      if(countc < 50 )
          countc <= countc + 1; // add 1 to the count until it get's to 50 counts. 
      else
          begin
          countc <= 0;
          sclk <= ~sclk; // ~ flips the value of sclk. If sclk is 1, then ~sclk will be 0, and if sclk is 0, then ~sclk will be 1. 
          This effectively toggles the signal between its current state and its complement.
          end
    end
  end
  
  ////// state machine
    reg [11:0] temp; // register with 12-bit.
    
  always@(posedge sclk) / always block to initiate positive edge for the clock. if posedge was negedge, then the clock was initiate in the negate edge.
  begin
    if(rst == 1'b1) begin 
      cs <= 1'b1; // Setting it to 1 typically means that no slave device is selected.
      mosi <= 1'b0; // It sets the initial value of mosi to 0 during a reset condition.
    end
    else begin
     case(state)
         idle:
             begin
               if(newd == 1'b1) begin // "high" indicating new data to be transmitted.
                 state <= send;
                 temp <= din; 
                 cs <= 1'b0; //  This line sets the cs signal to 0, selecting the slave device for communication.
               end
               else begin
                 state <= idle; // This line sets the state variable to idle, indicating that the SPI module remains in the idle state.
                 temp <= 8'h00;
               end
             end
                         
       send : begin
         if(count <= 11) begin
           mosi <= temp[count]; /////sending lsb first
           count <= count + 1;
         end
         else
             begin
               count <= 0;
               state <= idle;
               cs <= 1'b1;
               mosi <= 1'b0;
             end
       end
                  
      default : state <= idle; 
       
   endcase
  end 
 end
  
endmodule

// declare interface 
interface spi_if; 

  logic clk;
  logic newd;
  logic rst;
  logic [11:0] din;
  logic sclk;
  logic cs;
  logic mosi;
  
endinterface

// Transaction Class
class transaction;
  
  rand bit newd;
  rand bit [11:0] din;
  bit cs;
  bit mosi;
  
  function void display (input string tag);
    $display("[%0s] : DATA_NEW : %0b DIN : %0d CS : %b MOSI : %0b ", tag, newd, din, cs, mosi);    
  endfunction
  
  function transaction copy();
    copy = new();
    copy.newd = this.newd;
    copy.din = this.din;
    copy.cs = this.cs;
    copy.mosi = this.mosi;
  endfunction
  
endclass

// Generator Class
class generator;
  
  transaction tr; // define tr variable for transaction.
  mailbox #(transaction) mbx; // mailbox is data structure used for cummunication between different process.
  event done; // This member represents an event. Events are used to synchronize processes or tasks.
  done is the name of the event. indicate the completion of a certain task or operation.
  By using events, you can create synchronization points in your code, ensuring that different processes or tasks coordinate their actions appropriately.
  Events are commonly used in hardware verification and design environments to synchronize the behavior of different modules or components.
  This is an event that is triggered when the generator has completed generating transactions.
  int count = 0;
  event drvnext; // used to synchronize the generator with the driver.
  event sconext; // used to synchronize the generator with the scoreboard.
  
  function new(mailbox #(transaction) mbx);
    this.mbx = mbx;
    tr = new();
  endfunction
  
  task run(); // The run task is responsible for generating transactions and putting them into the mailbox.  
    repeat(count) begin // uses a repeat loop that runs count times.
      assert(tr.randomize) else $error("[GEN] :Randomization Failed"); // randomizes the tr transaction using the randomize method.
      If the randomization fails, it prints an error message.
      mbx.put(tr.copy); // The randomized transaction is then put into the mailbox using the put method. 
      tr.display("GEN"); // displays the transaction information using the display method By passing "GEN" as an argument,
      it indicates that this particular display statement is coming from the generator component.
      It helps identify the source or origin of the displayed information.
      @(drvnext); //  the task waits for the drvnext event and sconext event to synchronize with the driver and scoreboard, respectively.
      These events are used to coordinate the activities of different components in the verification environment.
      @(sconext);
    end
    -> done;
  endtask
  
endclass

// Driver Class
 
class driver;
  
  virtual spi_if vif; 
  mailbox #(bit [11:0]) mbxds; 
  event drvnext;  // used to synchronize he driver process with other processes or threads within the driver class. 
  bit [11:0] din;
 
  function new(mailbox #(bit [11:0]) mbxds, mailbox #(transaction) mbx); // make function to make the program easiar.
  automatically called when you create an object of the driver class.
    this.mbx = mbx;
    this.mbxds = mbxds;
    endfunction
  
  task reset(); // reset the take. useful to unitialize or reinitialize the state of task before excution.\\
     vif.rst <= 1'b1; // reset is on. 
     vif.cs <= 1'b1; // The slave dont get data from the master (he isn't selected).
     vif.newd <= 1'b0; // no new data is being presented to the interface.
     vif.din <= 1'b0; // no specific data is being transmitted.
     vif.mosi <= 1'b0; // no data from the master is being sent to the slave. 
    repeat(10) @(posedge vif.clk); // I want to repeat ten times on the reset because I want to verify uncover 
    potential issues or verify the correnctness of certain operations.
    vif.rst <= 1'b0; // take out the system from reset mode and get into operational mode. 
    repeat(5) @(posedge vif.clk); // I want to repeat five times to make stability for the system and dealy before the message of the RESET operate. 
    $display("[DRV] : RESET DONE"); // message that show that the reset is done.
  endtask
  
  task run();
    forever begin // make infinite loop until the simulation is stopped.
      mbx.get(tr); // call the data in the transaction class.
      @(posedge vif.sclk);
      vif.newd <= 1'b1; // put a new data.
      vif.din <= tr.din; // the interface gets the data from transaction class. 
      mbxds.put(tr.din); // I call to the data on mbxds mailbox. "put" is used to put a value into the mailbox.
      so, basically, I "put" data from transaction class into the mailbox.  
      @(posedge vif.sclk);
      vif.newd <= 1'b0;
      wait(vif.cs == 1'b1); // the usage of "wait" operator is to suspends the execution of the current task until the condition on the bracket take place. 
      $display("[DRV] : DATA SENT TO DAC : %0d",tr.din); // %0d used to display the decimal value of tr.din.
      ->drvnext; // I want to trigger this event so I can synchronize this task with other processes or threads within the driver class.
    end
    
  endtask

stores the data in another mailbox, waits for a condition to be true, displays a message, 
and triggers an event before repeating the process again. It represents a continuous operation of data transfer between components.
  
endclass

// Monitor Class
 
class monitor;
transaction tr; // need to extract the data from transaction class.
  mailbox #(bit [11:0]) mbx; // # is symbol that allow to specify the size or wide of the datatype used in the mailbox object.
  if I dont include #, i'll get error.
  bit [11:0] srx; // "srx" is being used to store the received data during monitoring process. in this class, the usage of this data register is to capture the received data bit by bit.
  
  virtual spi_if vif;
  
  function new(mailbox #(bit [11:0]) mbx); // making function for organize the object creation process.
  this.mbx = mbx;
  endfunction
  
  task run();
    
    forever begin
      @(posedge vif.sclk);
      wait(vif.cs == 1'b0); ///start of transaction
      @(posedge vif.sclk);
      
      for(int i=0; i<= 11; i++) begin 
        @(posedge vif.sclk); // the loop waits for positive edge before capturing each bit to ensure synchroniztion with the clock signal.
        srx[i] = vif.mosi; // store the data we receive bit by bit.         
      end
      
      wait(vif.cs == 1'b1);  ///end of transaction
      
      $display("[MON] : DATA SENT : %0d", srx);
      mbx.put(srx);
     end  
    
endtask
  
endclass
 
// Scoreboard Class
 
class scoreboard;
  mailbox #(bit [11:0]) mbxds, mbxms;
  bit [11:0] ds;
  bit [11:0] ms;
  event sconext;
  
  function new(mailbox #(bit [11:0]) mbxds, mailbox #(bit [11:0]) mbxms);
    this.mbxds = mbxds;
    this.mbxms = mbxms;
  endfunction
  
  task run();
    forever begin
      
      mbxds.get(ds);
      mbxms.get(ms);
      $display("[SCO] : DRV : %0d MON : %0d", ds, ms);
      if(ds == ms)
        $display("[SCO] : DATA MATCHED");
      else
        $display("[SCO] : DATA MISMATCHED");
      ->sconext;     
    end
  endtask

endclass

// Environment Class
 
class environment; 
 
    generator gen;
    driver drv;
    monitor mon;
    scoreboard sco; 
  
    event nextgd; ///gen -> drv, synchronize generator class with driver class
    event nextgs;  /// gen -> sco, synchronize generator class with driver scoreboard.
  
  mailbox #(transaction) mbxgd; ///gen - drv, communication between the generator and the drive - Different types of data can be passed through these 
  mailboxes for analysis and synchronization. 
  mailbox #(bit [11:0]) mbxds; /// drv - mon 
  mailbox #(bit [11:0]) mbxms;  /// mon - sco
  
  virtual spi_if vif; // connect the interface of SPI with envirement class. 
 
  function new(virtual spi_if vif);
       
    mbxgd = new(); // "new()" is being used to allocate memory for the mailbox and initializthe e it.
    it returns a reference to the created mailbox, which assigned to the varibale "mbxgd". 
    using this function ensure that a fresh mailbox is available for communication and that is prperly initialized before being used.
    if I dont use new(), the test between classe's wont be dynamic and flexibility. 
    mbxms = new();
    mbxds = new();
    gen = new(mbxgd);
    drv = new(mbxds,mbxgd);
 
    mon = new(mbxms);
    sco = new(mbxds, mbxms);
    
    this.vif = vif;
    drv.vif = this.vif;
    mon.vif = this.vif;
    
    gen.sconext = nextgs;
    sco.sconext = nextgs;
    
    gen.drvnext = nextgd;
    drv.drvnext = nextgd;
 
  endfunction
  
  task pre_test(); // initialize the test environment 
    drv.reset(); // reset the driver class before testing. 
  endtask
  
  task test(); // start making the test
  fork
    gen.run(); // start generator class 
    drv.run();
    mon.run();
    sco.run();
  join_any
  endtask
  
  task post_test(); // this line of code cause that the simulation will not continue inderfinitely.
  This can lead to unnecessary resource usage and make it difficult to analyze the simulation results or determine when the test has actually completed.
  task allows you to properly finalize the simulation, clean up any remaining tasks or processes, and exit gracefully.
  It ensures that the simulation terminates at the appropriate time and provides a clear indication of the end of the test phase. 
    wait(gen.done.triggered); // pause execution until gen is trigger. 
    $finish(); // call to end the simulation. 
  endtask
  
  task run();
    pre_test(); // This task orchestrates the overall test flow by calling other tasks in a sequence.
    The pre_test() task is called first to initialize the test environment, which may involve setting initial states or configurations.
    test(); // The test() task is then called using the fork-join_any construct.
    This launches concurrent execution of the gen.run(), drv.run(), mon.run(), and sco.run() tasks, allowing them to run simultaneously.
    post_test(); // task is called to perform post-processing and terminate the simulation.
  endtask

endclass

// Testbench Top

module tb;
    
  spi_if vif();
  
  spi dut(vif.clk,vif.newd,vif.rst,vif.din,vif.sclk,vif.cs,vif.mosi);
 
    initial begin
      vif.clk <= 0;
    end
    
    always #10 vif.clk <= ~vif.clk;
  
    environment env;
   
    initial begin
      env = new(vif);
      env.gen.count = 20;
      env.run();
    end
     
    initial begin
      $dumpfile("dump.vcd");
      $dumpvars;
    end  
  endmodule
 
 

 
 


