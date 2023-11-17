/*
 * lookup_tables.h
 *
 *  Created on: Nov. 17, 2023
 *      Author: Kenneth
 */

#pragma once

#include <stdint.h>

typedef enum {
    SCALE_TYPE_INT8 = 0,
    SCALE_TYPE_INT16
} scaleType_e;


//Handle circular reference. Reason explained below
typedef struct scale_t scale_t;
typedef struct scale_result_t scale_result_t;

typedef struct scale_functions {
    int (*Compare)(const void *, const void *);
    void (*SetValue)(const void *, const void *); 
    void * (*GetArrayValue)(void * data, uint8_t index); 
    void (*MapValue)(void * result, void * source, void * sourceLow, void * sourceHigh, void * mapLow, void * mapHigh);  
} scale_functions_t;    

//contains snapshot of scaling action given specified source value
//Thus actual scaling can be asyncronous to source value updates
//   if *scale->SourceValue == result.ScaledValue,  no need to re-scale.
//        
//
struct scale_result_t {
    scale_t * Scale;
    uint8_t IndexHigh;
    uint8_t IndexLow;
    void * ScaledValue;
};

// Defnies the 
struct scale_t {
    scaleType_e Type;  
    scale_result_t * Result;
    void * SourceValue;
    uint8_t Length;
    void * Data;
};

typedef struct {
    uint8_t Dimensions;
    void * Adder;
    scale_result_t * ScaledValue;
    void * Data;
} map_t;

void Scale(scale_t * scale, scale_functions_t * functions);
void MapLookup(map_t * map, void * result, scale_functions_t * functions);
void __scale_t_SetResult(scale_t * scale, scale_result_t * result);
void SetScaleFunctions_int(scale_functions_t * functions);
