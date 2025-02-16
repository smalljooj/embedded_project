# Biblioteca para o Sensor DS18B20

## Descrição Geral

Esta biblioteca foi desenvolvida para a comunicação com o sensor DS18B20 por meio do protocolo 1-Wire. Ela disponibiliza funções para inicialização do sensor, escrita e leitura de bits, leitura de endereços e obtenção da temperatura.

## Exemplo de código

    ds18b20_init();
    ds18b20_read_addresses();
    uint8_t address_count = ds18b20_get_address_count();
    float temperature;
    for(int i = 0; i < address_count; i++)
    {
        temperature = ds18b20_read_temperature_addr(CELSIUS, i);
        printf("Temperatura: %.2f °C\n", temperature);
    }
    printf("\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

### Funções

1. ds18b20_init()
Entrada: Nenhuma.
Saída: Nenhuma.
Descrição: Inicializa o sensor por meio da função ds18b20_restart() e configura o barramento como entrada.

2. ds18b20_restart()
Entrada: Nenhuma.
Saída: Retorna um valor indicando se o sensor foi reiniciado com sucesso.
Descrição: Coloca o barramento em estado LOW por 480µs, aguarda 70µs e lê o valor do barramento. Caso o valor lido seja LOW, o sensor foi corretamente reiniciado.

3. ds18b20_get_address_count()
Entrada: Nenhuma.
Saída: Retorna a quantidade de sensores detectados no barramento.
Descrição: Indica quantos sensores foram identificados após a execução da função ds18b20_read_addresses().

4. ds18b20_read_addresses()
Entrada:Nenhuma.
Saída:Nenhuma.
Descrição: Utiliza o comando Search ROM (0xF0) para identificar e armazenar os endereços dos sensores conectados ao barramento, resolvendo possíveis conflitos quando múltiplos sensores respondem simultaneamente.

5. ds18b20_adresses_print()
Entrada: Nenhuma.
Saída: Nenhuma.
Descrição: Imprime na saída padrão os endereços de todos os sensores detectados.

6. ds18b20_write_bit()
Entrada: Bit a ser escrito.
Saída: Nenhuma.
Descrição: Envia um bit no sensor.

7. ds18b20_write_bit()

Entrada: Byte a ser escrito.
Saída: Nenhuma.
Descrição: Envia um byte ao sensor.

8. ds18b20_read_bit()

Entrada: Nenhuma.
Saída: Retorna o bit lido do sensor.
Descrição: Realiza a leitura de um bit enviado pelo sensor.

9. ds18b20_read_byte()

Entrada: Nenhuma.
Saída: Retorna o byte lido do sensor.
Descrição: Realiza a leitura de um byte enviado pelo sensor.

10. ds18b20_read_temperature()
Entrada: Recebe uma unidade de temperatura (CELSIUS / KELVIN / FAHRENHEIT).
Saída: Retorna a temperatura do sensor na unidade escolhida.
Log:Exibe uma mensagem caso não encontre o sensor ou a conversão falhe.
Descrição: Realiza a leitura da temperatura quando há apenas um sensor no barramento. Utiliza os seguintes comandos

    SKIP ROM (0xCC): Evita a identificação de sensores individuais.

    Start Conversion (0x44): Solicita a medição da temperatura.

    Read Scratchpad (0xBE): Obtém os 9 bytes referentes à temperatura medida.
    Após armazenar os valores, a função retorna a temperatura na unidade desejada.

11. ds18b20_read_temperature_addr()
Entrada: Unidade de temperatura (CELSIUS / KELVIN / FAHRENHEIT) e índice do sensor (0 a 4).
Saída: Retorna a temperatura medida pelo sensor especificado.
Log: Exibe uma mensagem caso o sensor não seja encontrado ou a conversão falhe.
Descrição: Mede a temperatura de um sensor específico no barramento. O processo segue as etapas

    SKIP ROM (0xCC): Evita a identificação individual no início, economizando processamento.

    Start Conversion (0x44): Inicia a medição da temperatura.

    MATCH ROM (0x55): Identifica o sensor desejado através do endereço de 64 bits.

    Read Scratchpad (0xBE): Obtém os dados da temperatura medida.

# OBSERVAÇÃO

Antes de utilizar as funções que manipulam endereços, é necessário executar ds18b20_read_addresses(), pois caso contrário, os endereços não estarão armazenados corretamente.