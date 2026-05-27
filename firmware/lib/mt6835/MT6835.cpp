
#include "./MT6835.h"
#include <SimpleFOC.h>


MT6835::MT6835(SPISettings settings, int nCS) : settings(settings), nCS(nCS) {
    // nix
};

MT6835::~MT6835() {
    // nix
};



void MT6835::init(SPIClass* _spi) {
    spi = _spi;
    if (nCS >= 0)
        pinMode(nCS, OUTPUT);
    spi->begin();
};




float MT6835::getCurrentAngle(){
    return (float)readRawAngle21() / (float)MT6835_CPR * _2PI;
};



// CRC-8: polynomial X^8+X^2+X+1 = 0x07 (MT6835)
uint8_t MT6835::calcCRC8(uint8_t d0, uint8_t d1, uint8_t d2) {
    uint8_t crc = 0;
    uint8_t data[3] = {d0, d1, d2};
    for (int i = 0; i < 3; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// CRC-6: polynomial X^6+X+1 = 0x03 (MT6701)
uint8_t MT6835::calcCRC6(uint32_t data, int bits) {
    uint8_t crc = 0;
    for (int i = bits - 1; i >= 0; i--) {
        uint8_t msb = (crc >> 5) & 1;
        uint8_t input = (data >> i) & 1;
        crc = (crc << 1) & 0x3F;
        if (msb ^ input)
            crc ^= 0x03;
    }
    return crc;
}

uint32_t MT6835::readRawAngle21(){
    // Response arrives 2 bytes early (during command phase) and is shifted
    // right by 1 bit due to differential transceiver idle-HIGH on MISO.
    // Transfer 5 bytes to capture full CRC for either MT6835 or MT6701.
    if (nCS >= 0)
        digitalWrite(nCS, LOW);
    spi->beginTransaction(settings);
    uint8_t b0 = spi->transfer((MT6835_OP_ANGLE<<4) | (MT6835_REG_ANGLE1 >> 8));
    uint8_t b1 = spi->transfer(MT6835_REG_ANGLE1 & 0x00FF);
    uint8_t b2 = spi->transfer(0x00);
    uint8_t b3 = spi->transfer(0x00);
    uint8_t b4 = spi->transfer(0x00);
    spi->endTransaction();
    if (nCS >= 0)
        digitalWrite(nCS, HIGH);

    read_count++;

    // Undo the 1-bit right shift from differential transceiver idle-HIGH.
    // This reconstructs the original chip output byte stream.
    uint8_t d0 = (b0 << 1) | (b1 >> 7);
    uint8_t d1 = (b1 << 1) | (b2 >> 7);
    uint8_t d2 = (b2 << 1) | (b3 >> 7);
    uint8_t d3 = (b3 << 1) | (b4 >> 7);

    // --- Try MT6835: 21-bit angle + 3-bit status + CRC-8 ---
    // d0..d2 = [angle20:0][status2:0], d3 = CRC-8
    if (chip_type == CHIP_UNKNOWN || chip_type == CHIP_MT6835) {
        uint32_t angle = ((uint32_t)d0 << 13) | ((uint32_t)d1 << 5) | (d2 >> 3);
        uint8_t status = d2 & 0x07;
        uint8_t crc_recv = d3;
        uint8_t crc_calc = calcCRC8(d0, d1, d2);

        if (crc_calc == crc_recv) {
            if (chip_type == CHIP_UNKNOWN) chip_type = CHIP_MT6835;
            last_status = status;
            last_good_angle = angle;
            return angle;
        }
    }

    // --- Try MT6701: 14-bit angle + 4-bit status + CRC-6 ---
    // d0 = angle[13:6], d1 = [angle5:0][status3:2], d2 = [status1:0][CRC5:0]
    if (chip_type == CHIP_UNKNOWN || chip_type == CHIP_MT6701) {
        uint16_t angle14 = ((uint16_t)d0 << 6) | (d1 >> 2);
        uint8_t status = ((d1 & 0x03) << 2) | (d2 >> 6);
        uint8_t crc_recv = d2 & 0x3F;
        uint32_t crc_data = ((uint32_t)angle14 << 4) | status;
        uint8_t crc_calc = calcCRC6(crc_data, 18);

        if (crc_calc == crc_recv) {
            if (chip_type == CHIP_UNKNOWN) chip_type = CHIP_MT6701;
            last_status = status;
            // Scale 14-bit to 21-bit range so getCurrentAngle() math stays correct
            last_good_angle = (uint32_t)angle14 << 7;
            return last_good_angle;
        }
    }

    // Neither CRC passed — return last known good angle
    crc_errors++;
    return last_good_angle;
};




bool MT6835::setZeroFromCurrentPosition(){
    MT6835Command cmd;
    cmd.cmd = MT6835_OP_ZERO;
    cmd.addr = 0x000;
    transfer24(&cmd);
    return cmd.data == MT6835_WRITE_ACK;
};


/**
 * Wait 6s after calling this method
 */
bool MT6835::writeEEPROM(){
    delay(1); // wait at least 1ms
    MT6835Command cmd;
    cmd.cmd = MT6835_OP_PROG;
    cmd.addr = 0x000;
    transfer24(&cmd);
    return cmd.data == MT6835_WRITE_ACK;
};





uint8_t MT6835::getBandwidth(){
    MT6835Options5 opts = { .reg = readRegister(MT6835_REG_OPTS5) };
    return opts.bw;
};
void MT6835::setBandwidth(uint8_t bw){
    MT6835Options5 opts = { .reg = readRegister(MT6835_REG_OPTS5) };
    opts.bw = bw;
    writeRegister(MT6835_REG_OPTS5, opts.reg);
};

// uint8_t MT6835::getHysteresis(){
//     MT6835Options3 opts = { .reg = getOptions3() };
//     return opts.hyst;
// };
// void MT6835::setHysteresis(uint8_t hyst){
//     MT6835Options3 opts = { .reg = getOptions3() };
//     opts.hyst = hyst;
//     setOptions3(opts);
// };

// uint8_t MT6835::getRotationDirection(){
//     // MT6835Options3 opts = { .reg = getOptions3() };
//     // return opts.rot_dir;
// };
// void MT6835::setRotationDirection(uint8_t dir){
//     // MT6835Options3 opts = { .reg = getOptions3() };
//     // opts.rot_dir = dir;
//     // setOptions3(opts);
// };


uint16_t MT6835::getABZResolution(){
    uint8_t hi = readRegister(MT6835_REG_ABZ_RES1);
    MT6835ABZRes lo = {
			.reg = readRegister(MT6835_REG_ABZ_RES2)
	};
    return (hi << 6) | lo.abz_res_low;
};
void MT6835::setABZResolution(uint16_t res){
     uint8_t hi = (res >> 2);
    MT6835ABZRes lo = {
			.reg = readRegister(MT6835_REG_ABZ_RES2)
	};
    lo.abz_res_low = res & 0x3F;
    writeRegister(MT6835_REG_ABZ_RES1, hi);
    writeRegister(MT6835_REG_ABZ_RES2, lo.reg);
};



bool MT6835::isABZEnabled(){
    MT6835ABZRes lo = {
			.reg = readRegister(MT6835_REG_ABZ_RES2)
	};
    return lo.abz_off==0;
};
void MT6835::setABZEnabled(bool enabled){
    MT6835ABZRes lo = {
			.reg = readRegister(MT6835_REG_ABZ_RES2)
	};
    lo.abz_off = enabled?0:1;
    writeRegister(MT6835_REG_ABZ_RES2, lo.reg);
};



bool MT6835::isABSwapped(){
    MT6835ABZRes lo = {
			.reg = readRegister(MT6835_REG_ABZ_RES2)
	};
    return lo.ab_swap==1;
};
void MT6835::setABSwapped(bool swapped){
    MT6835ABZRes lo = {
			.reg = readRegister(MT6835_REG_ABZ_RES2)
	};
    lo.ab_swap = swapped?1:0;
    writeRegister(MT6835_REG_ABZ_RES2, lo.reg);
};



uint16_t MT6835::getZeroPosition(){
    uint8_t hi = readRegister(MT6835_REG_ZERO1);
    MT6835Options0 lo = {
            .reg = readRegister(MT6835_REG_ZERO2)
    };
    return (hi << 4) | lo.zero_pos_low;
};
void MT6835::setZeroPosition(uint16_t pos){
    uint8_t hi = (pos >> 4);
    MT6835Options0 lo = {
            .reg = readRegister(MT6835_REG_ZERO2)
    };
    lo.zero_pos_low = pos & 0x0F;
    writeRegister(MT6835_REG_ZERO1, hi);
    writeRegister(MT6835_REG_ZERO2, lo.reg);
};



MT6835Options1 MT6835::getOptions1(){
    MT6835Options1 result = {
			.reg = readRegister(MT6835_REG_OPTS1)
	};
    return result;
};
void MT6835::setOptions1(MT6835Options1 opts){
    writeRegister(MT6835_REG_OPTS1, opts.reg);
};



MT6835Options2 MT6835::getOptions2(){
    MT6835Options2 result = {
			.reg = readRegister(MT6835_REG_OPTS2)
	};
    return result;
};
void MT6835::setOptions2(MT6835Options2 opts){
    MT6835Options2 val = getOptions2();
    val.nlc_en = opts.nlc_en;
    val.pwm_fq = opts.pwm_fq;
    val.pwm_pol = opts.pwm_pol;
    val.pwm_sel = opts.pwm_sel;
    writeRegister(MT6835_REG_OPTS2, val.reg);
};



MT6835Options3 MT6835::getOptions3(){
    MT6835Options3 result = {
			.reg = readRegister(MT6835_REG_OPTS3)
	};
    return result;    
};
void MT6835::setOptions3(MT6835Options3 opts){
    MT6835Options3 val = getOptions3();
    val.rot_dir = opts.rot_dir;
    val.hyst = opts.hyst;
    writeRegister(MT6835_REG_OPTS3, val.reg);
};



MT6835Options4 MT6835::getOptions4(){
    MT6835Options4 result = {
			.reg = readRegister(MT6835_REG_OPTS4)
	};
    return result;
};
void MT6835::setOptions4(MT6835Options4 opts){
    MT6835Options4 val = getOptions4();
    val.gpio_ds = opts.gpio_ds;
    val.autocal_freq = opts.autocal_freq;
    writeRegister(MT6835_REG_OPTS4, val.reg);
};






void MT6835::transfer24(MT6835Command* outValue) {
    uint8_t b0 = (outValue->cmd << 4) | (outValue->addr >> 8);
    uint8_t b1 = outValue->addr & 0xFF;
    if (nCS >= 0)
        digitalWrite(nCS, LOW);
    spi->beginTransaction(settings);
    spi->transfer(b0);
    spi->transfer(b1);
    outValue->data = spi->transfer(0x00);
    spi->endTransaction();
    if (nCS >= 0)
        digitalWrite(nCS, HIGH);
};
uint8_t MT6835::readRegister(uint16_t reg) {
    MT6835Command cmd;
    cmd.cmd = MT6835_OP_READ;
    cmd.addr = reg;
    transfer24(&cmd);
    return cmd.data;
};
bool MT6835::writeRegister(uint16_t reg, uint8_t value) {
    MT6835Command cmd;
    cmd.cmd = MT6835_OP_READ;
    cmd.addr = reg;
    cmd.data = value;
    transfer24(&cmd);
    return cmd.data == MT6835_WRITE_ACK;
};