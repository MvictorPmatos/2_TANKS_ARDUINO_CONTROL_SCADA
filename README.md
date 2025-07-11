# Sistema de Automação e Supervisão de Dois Tanques – Arduino + SCADA

Este repositório contém os códigos-fonte utilizados no desenvolvimento de um sistema de automação e supervisão aplicado a uma planta didática composta por dois tanques, como parte do Trabalho de Conclusão de Curso (TCC) dos alunos Victor Matos e Gerardo Teixeira.

## Funcionalidades

- Leitura de nível dos tanques por sensores de pressão (MPX5010DP)
- Controle de bombas de porão via PWM
- Implementação de controle PID e realimentação linearizante
- Comunicação entre Arduinos e sistema supervisório (SCADA) via protocolo Modbus RTU (RS485)
- Troca de dados com o SCADA (setpoints, variáveis de processo, controle manual/automático)

## Estrutura dos Códigos

Os arquivos `.ino` estão divididos conforme as responsabilidades dos Arduinos:
- **Arduino 1**: Controle e monitoramento do tanque 1
- **Arduino 2**: Controle e monitoramento do tanque 2

## Requisitos

Para o correto funcionamento da comunicação Modbus via RS485, é necessário instalar a biblioteca:

📦 **[emelianov/modbus-esp8266](https://github.com/emelianov/modbus-esp8266)**

Essa biblioteca permite implementar o protocolo Modbus RTU sobre hardware compatível com ESP8266/ESP32, mas também funciona com Arduino UNO e Mega (via SoftwareSerial). Ela é essencial para leitura e escrita de registradores Modbus.
