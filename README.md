# Projeto PID

Este projeto Arduino implementa um controle PD para um atuador servo baseado em medições de distância por ultrassom e orientação de um sensor MPU-6050.

## Objetivo

Controlar um servo de forma a manter uma distância desejada entre o sensor ultrassônico e um obstáculo, além de atuar em modos de resgate quando o sensor detecta perda de referência ou proximidade extrema.

## Componentes principais

- Arduino compatível
- Sensor ultrassônico HC-SR04
- Servo motor
- Sensor MPU-6050 (acelerômetro/giroscópio)
- Cabos e fonte de alimentação adequada

## Funcionamento

1. Leitura do MPU-6050 para obter a inclinação atual da bandeja.
2. Medição de distância com o sensor ultrassônico.
3. Modo de resgate quando o sensor não detecta uma parede válida ou quando a distância é maior que 17,5 cm.
4. Modo de correção ao detectar proximidade excessiva (<= 11 cm) com a parede direita.
5. Controle PD para ajustar o servo e manter o setpoint de distância desejado.

## Parâmetros ajustáveis

- `setpoint`: distância desejada em centímetros (atualmente `19.0` cm)
- `Kp`: ganho proporcional (atualmente `1.8`)
- `Kd`: ganho derivativo (atualmente `2.5`)
- `Ki`: ganho integral, que está definido como `0.0`
- `alpha`: coeficiente do filtro exponencial para suavizar a leitura de distância

## Pinos utilizados

- `trigPin`: 10
- `echoPin`: 11
- `servoPin`: 9

## Saída serial

O código envia dados para o Serial Monitor em formato CSV para análise e plotagem:

- `Dist`: distância filtrada em cm
- `ErroX10`: erro multiplicado por 10 para melhor visualização
- `AnguloServo`: ângulo atual enviado ao servo
- `AnguloReal`: ângulo real calculado pelo MPU-6050

Exemplo de saída:

```
Dist:18.5,ErroX10:5.0,AnguloServo:55,AnguloReal:-1.2
```

## Observações

- O projeto usa um controlador PD, sem termo integral ativo.
- Há tratamento especial para perda de referência do sensor e situações de desatolamento.
- A faixa de ângulo do servo é limitada entre `20` e `80` graus.

## Como usar

1. Conecte o servo ao pino 9.
2. Conecte o trig do ultrassom ao pino 10 e o echo ao pino 11.
3. Conecte o MPU-6050 aos pinos I2C do Arduino (A4 - SDA, A5 - SCL).
4. Abra o Serial Monitor em `115200` bauds para visualizar os dados.
5. Faça ajustes em `Kp`, `Kd`, `setpoint` e `alpha` conforme necessário.