# Biblioteca para o Display OLED SSD1306

## Descrição Geral

Esta biblioteca foi projetada para configurar e escrever dados no display usando comunicação I²C. Ela fornece funções para inicializar o display, 
escrever bitmaps, texto, pontos, linhas, retangulos e triangulos.

## Exemplo de código

    oled_display_init();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // select the entire display
    oled_display_set_column_addresses(0, 127);
    oled_display_set_page_addresses(0, 7);

    oled_display_set_cursor(0, 0);
    oled_display_write_text("Hello, are you \nok?", 19);
    oled_display_draw_line(0, 25, 127, 25);
    oled_display_draw_rectangle(35, 35, 55, 55);
    oled_display_draw_triangle(70, 55, 100, 55, 85, 35);
    oled_display_update_buffer();
    while(1)
    {
        oled_display_check();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

## Funções

### `oled_display_init()`

Inicializa o display configurando os seguintes passos:

- Frequência do oscilador
- Deslocamento do display
- Linha de começo
- Charge Pump para que o display consiga funcionar com a alimentação de 3v3 a 5v
- Modo de enderaçamento Horizontal
- Remapeamento para ajustar orientação do display, tanto colunas como linhas
- Contraste do display

### `oled_display_write_cmd(uint8_t cmd)`

Envia comandos para o Display, como o comando de ligar o display:

`oled_display_write_cmd(0xAF);`

### `oled_display_write_data(uint8_t* data, unsigned long length)`

Envia dados para o Display, como o bitmaps e pontos.

### `oled_display_set_column_addresses(uint8_t start, uint8_t end)`

Define o inicio e fim das colunas do display, pode ser utilizado para delimitar o tamanho que será utilizado do display.

### `oled_display_set_page_addresses(uint8_t start, uint8_t end)`

Define o inicio e fim das paginas do display, as linhas do display são separadas em 8 paginas de 8, totalizando 64 linhas,
portanto se for necessário delimitar o display das linhas 8 a 55(pois se inicia no 0 e finda-se no 63), será necessário utilizar:

`oled_display_set_page_addresses(1, 6)`

### `oled_display_clear(void)`

Limpa a tela.

### `oled_display_fill(void)`

Preenche a tela de branco ou nível alto(a cor original do display).

### `oled_display_draw_pixel(uint8_t x, uint8_t y, uint8_t pixel_state)`

Desenha um pixel no buffer.

### `oled_display_update_buffer(void)`

Envia o buffer para a tela, pois após utilizar funções que utilizam o buffer é necessário atualizá-lo. As funções que utilizam são:

- oled_display_draw_pixel
- oled_display_write_text
- oled_display_draw_line
- oled_display_draw_rectangle
- oled_display_draw_triangle

### `oled_display_set_cursor(uint8_t x, uint8_t y)`

Define onde o cursor começará a escrever os caracteres da função `oled_display_write_txt`.

### `oled_display_write_text(char* txt, uint16_t length)`

Escreve caracteres no display utilizando uma fonte de 8x8.

### `oled_display_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)`

Desenha uma linha no display, utilizando dois pontos. 

Obs: Utiliza interpolação linear por reta paramétrica.

### `oled_display_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)`

Desenha um retângulo no display, utilizando pontos das diagonais do retângulo. 

Obs: Utiliza interpolação linear por reta paramétrica.

### `oled_display_draw_triangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3)`


Desenha um triângulo no display, utilizando os seus 3 pontos.

Obs: Utiliza interpolação bilinear. Portanto consome mais recurso...

### `oled_display_check()`

É utilizado para verificar se o display está conectado, e caso seja desconectado e reconectado, ele é reiniciado e o buffer atualizado.
