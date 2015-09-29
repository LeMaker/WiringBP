#!/bin/bash

# Example of how to export PWM Pin and play tones via CLI

export WIRINGPI_DEBUG=true 
export WIRINGPI_CODES=true 
GPIO=../gpio/gpio

$GPIO export 1 out
$GPIO mode 1 pwm
$GPIO pwmTone 1 1033

for i in 262 523 220 440 233 466 0 0 262 523 220 440 233 466 0 0 175 349 147 294 156 311 0 0 175 349 147 294 156 311 0 0 311 277 294 277 311 311 208 196 277 262 370 349 165 466 440 415 311 247 233 220 208 0 0 0 ; do 
	$GPIO pwmTone 1 $i;
	sleep 0.15;
done
