/*---------------------------------------------------------------------------------
  command line interface

  Credit : Mads Aasvik's Arduino Tutorial

  Reference:
  https://www.norwegiancreations.com/2018/02/creating-a-command-line-interface-in-arduinos-serial-monitor/

---------------------------------------------------------------------------------*/
#include "firmware.h"

int  cmd_help();
int  cmd_reg();
void help_help();
void help_reg();

#if CLI_FEATURE
#define LINE_BUF_SIZE   128     //Maximum input string length
#define ARG_BUF_SIZE    64      //Maximum argument string length
#define MAX_NUM_ARGS    8       //Maximum number of arguments
 
bool    error_flag = false;
 
char line[LINE_BUF_SIZE];
char args[MAX_NUM_ARGS][ARG_BUF_SIZE];

//List of functions pointers corresponding to each command
int (*commands_func[])(){
    &cmd_help,
    &cmd_reg
};
 
//List of command names
const char *commands_str[] = {
    "help",
    "reg",
};
 
int num_commands = sizeof(commands_str) / sizeof(char *);

void read_line(){
    String line_string;

 //   while(!Serial.available());
 
    if(Serial.available()){
        line_string = Serial.readStringUntil('\n');
        if(line_string.length() < LINE_BUF_SIZE){
          line_string.toCharArray(line, LINE_BUF_SIZE);
          Serial.println(line_string);
        }
        else{
          Serial.println("Input too long.");
          error_flag = true;
        }
    }
}
 
void parse_line(){
    char *argument;
    int counter = 0;
 
    argument = strtok(line, " ");
 
    while((argument != NULL)){
        if(counter < MAX_NUM_ARGS){
            if(strlen(argument) < ARG_BUF_SIZE){
                strcpy(args[counter],argument);
                argument = strtok(NULL, " ");
                counter++;
            }
            else{
                Serial.println("Input string too long.");
                error_flag = true;
                break;
            }
        }
        else{
            break;
        }
    }
}
 
int execute(){  
    for(int i=0; i<num_commands; i++){
        if(strcmp(args[0], commands_str[i]) == 0){
            return(*commands_func[i])();
        }
    }
 
    Serial.println("Invalid command. Type \"help\" for more.");
    return 0;
}

int cmd_help(){
    if(args[1] == NULL){
        help_help();
    }
    else if(strcmp(args[1], commands_str[0]) == 0){
        help_help();
    }
    else if(strcmp(args[1], commands_str[1]) == 0){
        help_reg();
    }
    else{
        help_help();
    }
}
 
void help_help(){
    Serial.println("The following commands are available:");
 
    for(int i=0; i<num_commands; i++){
        Serial.print("  ");
        Serial.println(commands_str[i]);
    }
    Serial.println("");
    Serial.println("You can for instance type \"help reg\" for more info on the reg command.");
}

//-----------------------------------------
void help_reg(){
    Serial.print("Set ADS1292R's register by \"reg adress value\n");
    Serial.println("  ");
}
 
extern void ecg_reg_set(uint8_t address, uint8_t data); 
int cmd_reg(){
    int8_t address, value;

    //address = atoi(args[1]);
    //value   = atoi(args[2]);

    sscanf (args[1],"%x",&address);
    sscanf (args[2],"%x",&value);

    //Serial.printf("set register @ %x = %x.\r\n", address, value);
    ecg_reg_set(address, value);
}
 
 
/*---------------------------------------------------------------------------------
 called from firmware.ino
---------------------------------------------------------------------------------*/
void handleCLI(){
   return;
    Serial.print("> ");
    
    read_line();
    if(!error_flag){
        parse_line();
    }
    if(!error_flag){
        execute();
    }
 
    memset(line, 0, LINE_BUF_SIZE);
    memset(args, 0, sizeof(args[0][0]) * MAX_NUM_ARGS * ARG_BUF_SIZE);
 
    error_flag = false;
}
#else
void handleCLI(){}
#endif //CLI_FEATURE