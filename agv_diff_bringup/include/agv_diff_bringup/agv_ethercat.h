#ifndef ARM_ETHERCAT
#define ARM_ETHERCAT

#include <stdbool.h>
#include "ecrt.h"


#define slave_num 4
#define NSLAVE 4

#define LiWei_Pulse_Pos  0, 0
#define LiWei_Pulse 0xe0000168, 0x0194196c



// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domainInput = NULL;
static ec_domain_state_t domainInput_state = {};

static ec_domain_t *domainOutput = NULL;
static ec_domain_state_t domainOutput_state = {};




static ec_slave_config_t *sc_pulse = NULL;
static ec_slave_config_state_t sc_pulse_state = {};

ec_slave_config_t *sc1;


// process data
static uint8_t *domainInput_pd = NULL;
static uint8_t *domainOutput_pd = NULL;



// offsets for PDO entries
static unsigned int off_control_word[4];
static unsigned int off_target_position[4];
static unsigned int off_digital_output[4];
static unsigned int off_target_velocity[4];
static unsigned int off_status_word[4];
static unsigned int off_actual_position[4];
static unsigned int off_actual_velocity[4];
static unsigned int off_digital_input[4];



const static ec_pdo_entry_reg_t domainOutput_regs[] = {
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x6040, 0, &off_control_word[0],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x607a, 0, &off_target_position[0],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x60fe, 1, &off_digital_output[0],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x60ff, 0, &off_target_velocity[0],NULL},

{LiWei_Pulse_Pos,  LiWei_Pulse, 0x6840, 0, &off_control_word[1],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x687a, 0, &off_target_position[1],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x68fe, 1, &off_digital_output[1],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x68ff, 0, &off_target_velocity[1],NULL},

{LiWei_Pulse_Pos,  LiWei_Pulse, 0x7040, 0, &off_control_word[2],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x707a, 0, &off_target_position[2],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x70fe, 1, &off_digital_output[2],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x70ff, 0, &off_target_velocity[2],NULL},

{LiWei_Pulse_Pos,  LiWei_Pulse, 0x7840, 0, &off_control_word[3],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x787a, 0, &off_target_position[3],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x78fe, 1, &off_digital_output[3],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x78ff, 0, &off_target_velocity[3],NULL},
{}
	};

const static ec_pdo_entry_reg_t domainInput_regs[] = {
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x6041, 0, &off_status_word[0],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x6064, 0, &off_actual_position[0],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x606c, 0, &off_actual_velocity[0],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x60fd, 0, &off_digital_input[0],NULL},

{LiWei_Pulse_Pos,  LiWei_Pulse, 0x6841, 0, &off_status_word[1],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x6864, 0, &off_actual_position[1],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x686c, 0, &off_actual_velocity[1],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x68fd, 0, &off_digital_input[1],NULL},

{LiWei_Pulse_Pos,  LiWei_Pulse, 0x7041, 0, &off_status_word[2],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x7064, 0, &off_actual_position[2],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x706c, 0, &off_actual_velocity[2],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x70fd, 0, &off_digital_input[2],NULL},

{LiWei_Pulse_Pos,  LiWei_Pulse, 0x7841, 0, &off_status_word[3],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x7864, 0, &off_actual_position[3],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x786c, 0, &off_actual_velocity[3],NULL},
{LiWei_Pulse_Pos,  LiWei_Pulse, 0x78fd, 0, &off_digital_input[3],NULL},			
	{}
	};
static unsigned int counter = 0;
static unsigned int curr_pos_[slave_num] = {0};
static unsigned int target_pos_[slave_num] = {0};

static ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Control Word */
    {0x607a, 0x00, 32}, /* Target Position */
    {0x60fe, 0x01, 32}, /* SubIndex 001 */
    {0x60ff, 0x00, 32}, /* Target Velocity */
    {0x6840, 0x00, 16}, /* Control Word */
    {0x687a, 0x00, 32}, /* Target Position */
    {0x68fe, 0x01, 32}, /* SubIndex 001 */
    {0x68ff, 0x00, 32}, /* Target Velocity */
    {0x7040, 0x00, 16}, /* Control Word */
    {0x707a, 0x00, 32}, /* Target Position */
    {0x70fe, 0x01, 32}, /* SubIndex 001 */
    {0x70ff, 0x00, 32}, /* Target Velocity */
    {0x7840, 0x00, 16}, /* Control Word */
    {0x787a, 0x00, 32}, /* Target Position */
    {0x78fe, 0x01, 32}, /* SubIndex 001 */
    {0x78ff, 0x00, 32}, /* Target Velocity */
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6064, 0x00, 32}, /* Position Actual Value */
    {0x606c, 0x00, 32}, /* Velocity Actual Value */
    {0x60fd, 0x00, 32}, /* SubIndex 000 */
    {0x6841, 0x00, 16}, /* Status Word */
    {0x6864, 0x00, 32}, /* Position Actual Value */
    {0x686c, 0x00, 32}, /* Velocity Actual Value */
    {0x68fd, 0x00, 32}, /* SubIndex 000 */
    {0x7041, 0x00, 16}, /* Status Word */
    {0x7064, 0x00, 32}, /* Position Actual Value */
    {0x706c, 0x00, 32}, /* Velocity Actual Value */
    {0x70fd, 0x00, 32}, /* SubIndex 000 */
    {0x7841, 0x00, 16}, /* Status Word */
    {0x7864, 0x00, 32}, /* Position Actual Value */
    {0x786c, 0x00, 32}, /* Velocity Actual Value */
    {0x78fd, 0x00, 32}, /* SubIndex 000 */

};

static ec_pdo_info_t slave_0_pdos[] = {
    {0x1601, 4, slave_0_pdo_entries + 0}, /* csp RxPDO */
    {0x1611, 4, slave_0_pdo_entries + 4}, /* csp RxPDO */
    {0x1621, 4, slave_0_pdo_entries + 8}, /* csp RxPDO */
    {0x1631, 4, slave_0_pdo_entries + 12}, /* csp RxPDO */
    {0x1a01, 4, slave_0_pdo_entries + 16}, /* csp TxPDO */
    {0x1a11, 4, slave_0_pdo_entries + 20}, /* csp TxPDO */
    {0x1a21, 4, slave_0_pdo_entries + 24}, /* csp TxPDO */
    {0x1a31, 4, slave_0_pdo_entries + 28}, /* csp TxPDO */

};

static ec_sync_info_t slave_0_syncs[] = {
	{0, EC_DIR_OUTPUT, 0, NULL,EC_WD_DISABLE},
	{1, EC_DIR_INPUT, 0, NULL,EC_WD_DISABLE},
	{2, EC_DIR_OUTPUT, 4, slave_0_pdos+0,EC_WD_ENABLE},
	{3, EC_DIR_INPUT, 4, slave_0_pdos+4,EC_WD_DISABLE},
	{0xff}
};




bool igh_configure();
bool igh_start();
int *igh_update(int *);
int *igh_get_curr_pos();
int *igh_get_home_pos();
int *igh_get_curr_vel();
int *igh_get_curr_switch_state();
void igh_stop();
void igh_cleanup();
void ini_driver(void);
void check_domain_state();
void check_master_state();
void check_slave_config_states();

int eth_curr_pos_[NSLAVE];
int eth_home_pos_[NSLAVE];
int eth_curr_vel_[NSLAVE];
int eth_tar_vel_[NSLAVE];
int state_[NSLAVE];
int loop_flag = 0;
int switch_state_[2] = {0, 0};

//RT_TASK my_task;
#endif
