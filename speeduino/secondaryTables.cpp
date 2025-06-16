#include "globals.h"
#include "secondaryTables.h"
#include "corrections.h"

void calculateSecondaryFuel(void)
{
  //If the secondary fuel table is in use, also get the VE value from there
  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Clear the bit indicating that the 2nd fuel table is in use. 
  if(configPage10.fuel2Mode > 0)
  { 
    if(configPage10.fuel2Mode == FUEL2_MODE_MULTIPLY)
    {
      currentStatus.VE2 = getVE2();
      //Fuel 2 table is treated as a % value. Table 1 and 2 are multiplied together and divided by 100
      uint16_t combinedVE = ((uint16_t)currentStatus.VE1 * (uint16_t)currentStatus.VE2) / 100;
      if(combinedVE <= UINT8_MAX) { currentStatus.VE = combinedVE; }
      else { currentStatus.VE = UINT8_MAX; }
    }
    else if(configPage10.fuel2Mode == FUEL2_MODE_ADD)
    {
      currentStatus.VE2 = getVE2();
      //Fuel tables are added together, but a check is made to make sure this won't overflow the 8-bit VE value
      uint16_t combinedVE = (uint16_t)currentStatus.VE1 + (uint16_t)currentStatus.VE2;
      if(combinedVE <= UINT8_MAX) { currentStatus.VE = combinedVE; }
      else { currentStatus.VE = UINT8_MAX; }
    }
    else if(configPage10.fuel2Mode == FUEL2_MODE_CONDITIONAL_SWITCH )
    {
      if(configPage10.fuel2SwitchVariable == FUEL2_CONDITION_RPM)
      {
        if(currentStatus.RPM > configPage10.fuel2SwitchValue)
        {
          BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use. 
          currentStatus.VE2 = getVE2();
          currentStatus.VE = currentStatus.VE2;
        }
      }
      else if(configPage10.fuel2SwitchVariable == FUEL2_CONDITION_MAP)
      {
        if(currentStatus.MAP > configPage10.fuel2SwitchValue)
        {
          BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use. 
          currentStatus.VE2 = getVE2();
          currentStatus.VE = currentStatus.VE2;
        }
      }
      else if(configPage10.fuel2SwitchVariable == FUEL2_CONDITION_TPS)
      {
        if(currentStatus.TPS > configPage10.fuel2SwitchValue)
        {
          BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use. 
          currentStatus.VE2 = getVE2();
          currentStatus.VE = currentStatus.VE2;
        }
      }
      else if(configPage10.fuel2SwitchVariable == FUEL2_CONDITION_ETH)
      {
        if(currentStatus.ethanolPct > configPage10.fuel2SwitchValue)
        {
          BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use. 
          currentStatus.VE2 = getVE2();
          currentStatus.VE = currentStatus.VE2;
        }
      }
    }
    else if(configPage10.fuel2Mode == FUEL2_MODE_INPUT_SWITCH)
    {
      if(digitalRead(pinFuel2Input) == configPage10.fuel2InputPolarity)
      {
        BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use. 
        currentStatus.VE2 = getVE2();
        currentStatus.VE = currentStatus.VE2;
      }
    }
  }
}


void calculateSecondarySpark(void)
{
  //Same as above but for the secondary ignition table
  BIT_CLEAR(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); //Clear the bit indicating that the 2nd spark table is in use. 
  if(configPage10.spark2Mode > 0)
  { 
    if(configPage10.spark2Mode == SPARK2_MODE_MULTIPLY)
    {
      BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE);
      currentStatus.advance2 = getAdvance2();
      //make sure we don't have a negative value in the multiplier table (sharing a signed 8 bit table)
      if(currentStatus.advance2 < 0) { currentStatus.advance2 = 0; }
      //Spark 2 table is treated as a % value. Table 1 and 2 are multiplied together and divided by 100
      int16_t combinedAdvance = ((int16_t)currentStatus.advance1 * (int16_t)currentStatus.advance2) / 100;
      //make sure we don't overflow and accidentally set negative timing, currentStatus.advance can only hold a signed 8 bit value
      if(combinedAdvance <= 127) { currentStatus.advance = combinedAdvance; }
      else { currentStatus.advance = 127; }
    }
    else if(configPage10.spark2Mode == SPARK2_MODE_ADD)
    {
      BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); //Set the bit indicating that the 2nd spark table is in use. 
      currentStatus.advance2 = getAdvance2();
      //Spark tables are added together, but a check is made to make sure this won't overflow the 8-bit VE value
      int16_t combinedAdvance = (int16_t)currentStatus.advance1 + (int16_t)currentStatus.advance2;
      //make sure we don't overflow and accidentally set negative timing, currentStatus.advance can only hold a signed 8 bit value
      if(combinedAdvance <= 127) { currentStatus.advance = combinedAdvance; }
      else { currentStatus.advance = 127; }
    }
    else if(configPage10.spark2Mode == SPARK2_MODE_CONDITIONAL_SWITCH )
    {
      if(configPage10.spark2SwitchVariable == SPARK2_CONDITION_RPM)
      {
        if(currentStatus.RPM > configPage10.spark2SwitchValue)
        {
          BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); //Set the bit indicating that the 2nd spark table is in use. 
          currentStatus.advance2 = getAdvance2();
          currentStatus.advance = currentStatus.advance2;
        }
      }
      else if(configPage10.spark2SwitchVariable == SPARK2_CONDITION_MAP)
      {
        if(currentStatus.MAP > configPage10.spark2SwitchValue)
        {
          BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); //Set the bit indicating that the 2nd spark table is in use. 
          currentStatus.advance2 = getAdvance2();
          currentStatus.advance = currentStatus.advance2;
        }
      }
      else if(configPage10.spark2SwitchVariable == SPARK2_CONDITION_TPS)
      {
        if(currentStatus.TPS > configPage10.spark2SwitchValue)
        {
          BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); //Set the bit indicating that the 2nd spark table is in use. 
          currentStatus.advance2 = getAdvance2();
          currentStatus.advance = currentStatus.advance2;
        }
      }
      else if(configPage10.spark2SwitchVariable == SPARK2_CONDITION_ETH)
      {
        if(currentStatus.ethanolPct > configPage10.spark2SwitchValue)
        {
          BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); //Set the bit indicating that the 2nd spark table is in use. 
          currentStatus.advance2 = getAdvance2();
          currentStatus.advance = currentStatus.advance2;
        }
      }
      else if(configPage10.spark2SwitchVariable == SPARK2_CONDITION_GEAR)   //[PJSC v1.10]
      {                                                                     // |
        if(currentStatus.gear > configPage10.spark2SwitchValue)             // |
        {                                                                   // |
          BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE);        // |Set the bit indicating that the 2nd spark table is in use. 
          currentStatus.advance2 = getAdvance2();                           // |
          currentStatus.advance = currentStatus.advance2;                   // |
        }                                                                   // V
      }                                                                     //[PJSC v1.10]
    }
    else if(configPage10.spark2Mode == SPARK2_MODE_INPUT_SWITCH)
    {
      if(digitalRead(pinSpark2Input) == configPage10.spark2InputPolarity)
      {
        BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); //Set the bit indicating that the 2nd spark table is in use. 
        currentStatus.advance2 = getAdvance2();
        currentStatus.advance = currentStatus.advance2;
      }
    }

    //Apply the fixed timing correction manually. This has to be done again here if any of the above conditions are met to prevent any of the seconadary calculations applying instead of fixec timing
    currentStatus.advance = correctionFixedTiming(currentStatus.advance);
    currentStatus.advance = correctionCrankingFixedTiming(currentStatus.advance); //This overrides the regular fixed timing, must come last
  }
}

/**
 * @brief Looks up and returns the VE value from the secondary fuel table
 * 
 * This performs largely the same operations as getVE() however the lookup is of the secondary fuel table and uses the secondary load source
 * @return byte 
 */
byte getVE2(void)
{
  byte tempVE = 100;
  if( configPage10.fuel2Algorithm == LOAD_SOURCE_MAP)
  {
    //Speed Density
    currentStatus.fuelLoad2 = currentStatus.MAP;
  }
  else if (configPage10.fuel2Algorithm == LOAD_SOURCE_TPS)
  {
    //Alpha-N
    currentStatus.fuelLoad2 = currentStatus.TPS * 2;
  }
  else if (configPage10.fuel2Algorithm == LOAD_SOURCE_IMAPEMAP)
  {
    //IMAP / EMAP
    currentStatus.fuelLoad2 = ((int16_t)currentStatus.MAP * 100U) / currentStatus.EMAP;
  }
  else { currentStatus.fuelLoad2 = currentStatus.MAP; } //Fallback position
  tempVE = get3DTableValue(&fuelTable2, currentStatus.fuelLoad2, currentStatus.RPM); //Perform lookup into fuel map for RPM vs MAP value

  return tempVE;
}

/**
 * @brief Performs a lookup of the second ignition advance table. The values used to look this up will be RPM and whatever load source the user has configured
 * 
 * @return byte The current target advance value in degrees
 */
byte getAdvance2(void)
{
  byte tempAdvance = 0;
  if (configPage10.spark2Algorithm == LOAD_SOURCE_MAP) //Check which fuelling algorithm is being used
  {
    //Speed Density
    currentStatus.ignLoad2 = currentStatus.MAP;
  }
  else if(configPage10.spark2Algorithm == LOAD_SOURCE_TPS)
  {
    //Alpha-N
    currentStatus.ignLoad2 = currentStatus.TPS * 2;

  }
  else if (configPage10.spark2Algorithm == LOAD_SOURCE_IMAPEMAP)
  {
    //IMAP / EMAP
    currentStatus.ignLoad2 = (currentStatus.MAP * 100) / currentStatus.EMAP;
  }
  else { currentStatus.ignLoad2 = currentStatus.MAP; }
  tempAdvance = get3DTableValue(&ignitionTable2, currentStatus.ignLoad2, currentStatus.RPM) - OFFSET_IGNITION; //As above, but for ignition advance

  //Perform the corrections calculation on the secondary advance value, only if it uses a switched mode
  if( (configPage10.spark2SwitchVariable == SPARK2_MODE_CONDITIONAL_SWITCH) || (configPage10.spark2SwitchVariable == SPARK2_MODE_INPUT_SWITCH) ) { tempAdvance = correctionsIgn(tempAdvance); } 

  return tempAdvance;
}

//[PJSC v1.10] *************************************************************************************
byte getVE3(void)
{
  byte tempVE = 100;
  if (configPage2.ignAlgorithm == LOAD_SOURCE_MAP) //Check which fuelling algorithm is being used
  {
    //Speed Density
    currentStatus.ignLoad = currentStatus.MAP;
  }
  else if(configPage2.ignAlgorithm == LOAD_SOURCE_TPS)
  {
    //Alpha-N
    currentStatus.ignLoad = currentStatus.TPS * 2;

  }
  else if (configPage2.fuelAlgorithm == LOAD_SOURCE_IMAPEMAP)
  {
    //IMAP / EMAP
    currentStatus.ignLoad = (currentStatus.MAP * 100) / currentStatus.EMAP;
  }
  else { currentStatus.ignLoad = currentStatus.MAP; } //Fallback position
  tempVE = get3DTableValue(&ignitionTable, currentStatus.ignLoad, currentStatus.RPM); //As above, but for ignition advance

  return tempVE;
}

byte getVE4(void)
{
  byte tempVE = 100;
  if (configPage10.spark2Algorithm == LOAD_SOURCE_MAP) //Check which fuelling algorithm is being used
  {
    //Speed Density
    currentStatus.ignLoad2 = currentStatus.MAP;
  }
  else if(configPage10.spark2Algorithm == LOAD_SOURCE_TPS)
  {
    //Alpha-N
    currentStatus.ignLoad2 = currentStatus.TPS * 2;

  }
  else if (configPage10.spark2Algorithm == LOAD_SOURCE_IMAPEMAP)
  {
    //IMAP / EMAP
    currentStatus.ignLoad2 = (currentStatus.MAP * 100) / currentStatus.EMAP;
  }
  else { currentStatus.ignLoad2 = currentStatus.MAP; }
  tempVE = get3DTableValue(&ignitionTable2, currentStatus.ignLoad2, currentStatus.RPM); //As above, but for ignition advance

  return tempVE;
}

byte calculateCombinedVE(byte primaryVE, byte secondaryVE)
{
  byte calcVE = primaryVE;

  //If the secondary fuel table is in use, also get the VE value from there
  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Clear the bit indicating that the 2nd fuel table is in use. 
  if(configPage10.fuel2Mode > 0)
  { 
    if(configPage10.fuel2Mode == FUEL2_MODE_MULTIPLY)
    {
      uint16_t combinedVE = ((uint16_t)primaryVE * (uint16_t)secondaryVE) / 100;
      if(combinedVE <= 255) { calcVE = combinedVE; }
      else { calcVE = 255; }
    }
    else if(configPage10.fuel2Mode == FUEL2_MODE_ADD)
    {
      uint16_t combinedVE = (uint16_t)primaryVE + (uint16_t)secondaryVE;
      if(combinedVE <= 255) { calcVE = combinedVE; }
      else { calcVE = 255; }
    }
    else if(configPage10.fuel2Mode == FUEL2_MODE_CONDITIONAL_SWITCH )
    {
      if(configPage10.fuel2SwitchVariable == FUEL2_CONDITION_RPM)
      {
        if(currentStatus.RPM > configPage10.fuel2SwitchValue)
        {
          BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use. 
          calcVE = secondaryVE;
        }
      }
      else if(configPage10.fuel2SwitchVariable == FUEL2_CONDITION_MAP)
      {
        if(currentStatus.MAP > configPage10.fuel2SwitchValue)
        {
          BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use. 
          calcVE = secondaryVE;
        }
      }
      else if(configPage10.fuel2SwitchVariable == FUEL2_CONDITION_TPS)
      {
        if(currentStatus.TPS > configPage10.fuel2SwitchValue)
        {
          BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use. 
          calcVE = secondaryVE;
        }
      }
      else if(configPage10.fuel2SwitchVariable == FUEL2_CONDITION_ETH)
      {
        if(currentStatus.ethanolPct > configPage10.fuel2SwitchValue)
        {
          BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use. 
          calcVE = secondaryVE;
        }
      }
    }
    else if(configPage10.fuel2Mode == FUEL2_MODE_INPUT_SWITCH)
    {
      if(digitalRead(pinFuel2Input) == configPage10.fuel2InputPolarity)
      {
        BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use. 
        calcVE = secondaryVE;
      }
    }
  }
  else
  {
    calcVE = primaryVE;
  }

  return calcVE;
}

void selectVEtable(void)
{
  byte tempPrimVE;
  byte tempSecVE;

  //currentStatus.VE1 = getVE1();
  currentStatus.VE2 = getVE2();
  currentStatus.VE3 = getVE3();
  currentStatus.VE4 = getVE4();

  if( configPage15.mapSwitchingEnabled && currentStatus.mapSelectSw )
  {
    tempPrimVE = selectVEvalue(configPage15.veMapSelectionInj1_2Pri);
    tempSecVE = selectVEvalue(configPage15.veMapSelectionInj1_2Sec);
  }
  else
  {
    tempPrimVE = selectVEvalue(configPage15.veMapSelectionInj1Pri);
    tempSecVE = selectVEvalue(configPage15.veMapSelectionInj1Sec);
  }
  currentStatus.inj1VE = calculateCombinedVE(tempPrimVE, tempSecVE);
  currentStatus.VE = currentStatus.inj1VE;

  if( configPage15.mapSeparationEnabled )
  {
    if( configPage15.mapSwitchingEnabled && currentStatus.mapSelectSw )
    {
      tempPrimVE = selectVEvalue(configPage15.veMapSelectionInj2_2Pri);
      tempSecVE = selectVEvalue(configPage15.veMapSelectionInj2_2Sec);
    }
    else
    {
      tempPrimVE = selectVEvalue(configPage15.veMapSelectionInj2Pri);
      tempSecVE = selectVEvalue(configPage15.veMapSelectionInj2Sec);
    }
    currentStatus.inj2VE = calculateCombinedVE(tempPrimVE, tempSecVE);

    if( configPage15.mapSwitchingEnabled && currentStatus.mapSelectSw )
    {
      tempPrimVE = selectVEvalue(configPage15.veMapSelectionInj3_2Pri);
      tempSecVE = selectVEvalue(configPage15.veMapSelectionInj3_2Sec);
    }
    else
    {
      tempPrimVE = selectVEvalue(configPage15.veMapSelectionInj3Pri);
      tempSecVE = selectVEvalue(configPage15.veMapSelectionInj3Sec);
    }
    currentStatus.inj3VE = calculateCombinedVE(tempPrimVE, tempSecVE);

    if( configPage15.mapSwitchingEnabled && currentStatus.mapSelectSw )
    {
      tempPrimVE = selectVEvalue(configPage15.veMapSelectionInj4_2Pri);
      tempSecVE = selectVEvalue(configPage15.veMapSelectionInj4_2Sec);
    }
    else
    {
      tempPrimVE = selectVEvalue(configPage15.veMapSelectionInj4Pri);
      tempSecVE = selectVEvalue(configPage15.veMapSelectionInj4Sec);
    }
    currentStatus.inj4VE = calculateCombinedVE(tempPrimVE, tempSecVE);
  }
  else
  {
    currentStatus.inj2VE = currentStatus.inj1VE;
    currentStatus.inj3VE = currentStatus.inj1VE;
    currentStatus.inj4VE = currentStatus.inj1VE;
  }
}

byte selectVEvalue(byte selectedVe)
{
  byte selectedVEvalue = currentStatus.VE;

  switch (selectedVe) {
    case SELECT_VE1:
      selectedVEvalue = currentStatus.VE1;
      break;

    case SELECT_VE2:
      selectedVEvalue = currentStatus.VE2;
      break;

    case SELECT_VE3:
      selectedVEvalue = currentStatus.VE3;
      break;

    case SELECT_VE4:
      selectedVEvalue = currentStatus.VE4;
      break;

    default:
      selectedVEvalue = currentStatus.VE;
      break;
  }

  return selectedVEvalue;
}
//[PJSC v1.10] *************************************************************************************
