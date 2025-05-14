section .note.GNU-stack noalloc noexec nowrite progbits
section .data               
;Cambiar Nombre y Apellido por vuestros datos.
developer db "Angel Torres",0

;Constante que también está definida en C.
DIMMATRIX equ 10
STONESYMBOLPLAYER1 equ 'X'
STONESYMBOLPLAYER2 equ 'O'

section .text            
;Variables definidas en ensamblador.
global developer                        

;Subrutinas de ensamblador que se llaman desde C.
global posCurBoardP2, showStonePosP2, moveCursorP2, checkAroundP2,
global insertStoneP2, checkRowP2, checkEndP2, playP2

;Funciones de C que se llaman desde ensamblador.
extern gotoxyP2_C, printchP2_C, getchP2_C
extern printMessageP2_C

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ATENCIÓN: Recordad que en ensamblador las variables y los parámetros 
;;   de tipo 'char' se deben asignar a registros de tipo  
;;   BYTE (1 byte): al, ah, bl, bh, cl, ch, dl, dh, sil, dil, ..., r15b
;;   los de tipo 'short' se deben asignar a registros de tipo 
;;   WORD (2 bytes): ax, bx, cx, dx, si, di, ...., r15w
;;   los de tipo 'int' se deben asignar a registros de tipo 
;;   DWORD (4 bytes): eax, ebx, ecx, edx, esi, edi, ...., r15d
;;   los de tipo 'long' se deben asignar a registros de tipo 
;;   QWORD (8 bytes): rax, rbx, rcx, rdx, rsi, rdi, ...., r15
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Las subrutinas en ensamblador que se deben modificar para
;; implementar el paso de parámetros son:
;;   posCurBoardP2, showStonePosP2, moveCursorP2
;;   checkAroundP2, insertStoneP2
;; La subrutina que se debe modificar la funcionalidad:
;;   checkEndP2
;; La subrutina que se debe implementar:
;;   checkRowP2
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Esta subrutina se da hecha. NO LA PODEIS MODIFICAR.
; Situar el cursor en una fila y una columna de la pantalla
; en función de la fila (edi) y de la columna (esi) recibidos como 
; parámetro llamando a la función gotoxyP2_C.
; 
; Variables globales utilizadas:	
; Ninguna
; 
; Parámetros de entrada: 
; (rowScreen): rdi(edi) : Fila de la pantalla donde se sitúa el cursor.
; (colScreen): rsi(esi) : Columna de la pantalla donde se sitúa el cursor.
;
; Parámetros de salida: 
; Ninguno
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
gotoxyP2:
   push rbp
   mov  rbp, rsp
   ;guardamos el estado de los registros del procesador porque
   ;las funciones de C no mantienen el estado de los registros.
   push rax
   push rbx
   push rcx
   push rdx
   push rsi
   push rdi
   push r8
   push r9
   push r10
   push r11
   push r12
   push r13
   push r14
   push r15

   ; Cuando llamamos a la función gotoxyP2_C(int row_num, int col_num) desde ensamblador 
   ; el primer parámetro (row_num) debe pasarse por el registro rdi(edi), y
   ; el segundo  parámetro (col_num) debe pasarse por el registro rsi(esi).	
   call gotoxyP2_C
 
   ;restaurar el estado de los registros que se han guardado en la pila.
   pop r15
   pop r14
   pop r13
   pop r12
   pop r11
   pop r10
   pop r9
   pop r8
   pop rdi
   pop rsi
   pop rdx
   pop rcx
   pop rbx
   pop rax

   mov rsp, rbp
   pop rbp
   ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Esta subrutina se da hecha. NO LA PODEIS MODIFICAR.
; Mostrar un carácter (dil) en la pantalla, recibido como parámetro, 
; en la posición donde está el cursor llamando a la función printchP2_C.
; 
; Variables globales utilizadas:	
; Ninguna
; 
; Parámetros de entrada: 
; (c) : rdi(dil) : Carácter a mostrar.
; 
; Parámetros de salida: 
; Ninguno
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
printchP2:
   push rbp
   mov  rbp, rsp
   ;guardamos el estado de los registros del procesador porque
   ;las funciones de C no mantienen el estado de los registros.
   push rax
   push rbx
   push rcx
   push rdx
   push rsi
   push rdi
   push r8
   push r9
   push r10
   push r11
   push r12
   push r13
   push r14
   push r15

   ; Cuando llamamos a la función printchP2_C(char c) desde ensamblador, 
   ; el parámetro (c) debe pasarse por el registro rdi(dil).
   call printchP2_C
 
   ;restaurar el estado de los registros que se han guardado en la pila.
   pop r15
   pop r14
   pop r13
   pop r12
   pop r11
   pop r10
   pop r9
   pop r8
   pop rdi
   pop rsi
   pop rdx
   pop rcx
   pop rbx
   pop rax

   mov rsp, rbp
   pop rbp
   ret
   

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Esta subrutina se da hecha. NO LA PODEIS MODIFICAR.
; Leer una tecla y retornar el carácter asociado (al) sin 
; mostrarlo por pantalla, llamando a la función getchP2_C.
; 
; Variables globales utilizadas:	
; Ninguna
; 
; Parámetros de entrada: 
; Ninguno
; 
; Parámetros de salida: 
; (c) : rax(al) : Carácter leído desde el teclado.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
getchP2:
   push rbp
   mov  rbp, rsp
   ;guardamos el estado de los registros del procesador porque
   ;las funciones de C no mantienen el estado de los registros.
   push rbx
   push rcx
   push rdx
   push rsi
   push rdi
   push r8
   push r9
   push r10
   push r11
   push r12
   push r13
   push r14
   push r15
   
   mov rax, 0
   ; Cuando llamamos a la función getchP2_C desde ensamblador, 
   ; retorna sobre el registro rax(al) el carácter leído
   call getchP2_C
 
   ;restaurar el estado de los registros que se han guardado en la pila.
   pop r15
   pop r14
   pop r13
   pop r12
   pop r11
   pop r10
   pop r9
   pop r8
   pop rdi
   pop rsi
   pop rdx
   pop rcx
   pop rbx
   
   mov rsp, rbp
   pop rbp
   ret 


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Posiciona el cursor en el tablero en la posición del cursor.
; Posicionar el cursor en el tablero en función de la variable
; (posCursor) llamando la subrutina gotoxyP2.
; La fila donde posicionamos el cursor (rowScreen) se calcula con la fórmula:
; (rowScreen = 7 + (posCursor/DIMMATRIX))*2
; La columna donde posicionamos el cursor (colScreen) se calcula con la fórmula:
; (colScreen = 8 + (posCursor%DIMMATRIX))*4
;  
; Variables globales utilizadas:	
; Ninguna.
; 
; Parámetros de entrada:
; (posCusor):rdi(rdi): Posición del cursor dentro de la matriz mBoard.
; 
; Parámetros de salida: 
; Ninguno.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;EX1 - Se ajustará el código de PR1 para que responda a posCurBOardP2.
;(En la medida de lo posible) 
section .text
; Subrutina posCurBoardP2
; Posiciona el cursor en el tablero según posCursor (en rdi).
; Fórmulas: rowScreen = 7 + (posCursor / DIMMATRIX) * 2
;           colScreen = 8 + (posCursor % DIMMATRIX) * 4
; Llama a gotoxyP2 para posicionar el cursor.
posCurBoardP2:
   push rbp                 ; Guardamos registro base en pila
   mov  rbp, rsp            ; rbp = rsp
   ; Guardar registros que se modifican
   push rax                 ; Guardamos rax (usado para cálculos)
   push rbx                 ; Guardamos rbx (para DIMMATRIX)
   push rdx                 ; Guardamos rdx (para resto de división)

   ; Cargar posCursor desde rdi
   mov rax, rdi             ; rax = posCursor

   ; Calcular rowScreen = 7 + (posCursor / DIMMATRIX) * 2
   mov rbx, DIMMATRIX       ; rbx = DIMMATRIX (10)
   xor rdx, rdx             ; Limpiar rdx para división
   div rbx                  ; rax = posCursor / DIMMATRIX, rdx = posCursor % DIMMATRIX
   shl rax, 1               ; rax = (posCursor / DIMMATRIX) * 2
   add rax, 7               ; rax = 7 + (posCursor / DIMMATRIX) * 2
   mov rbx, rax             ; Guardar rowScreen en rbx temporalmente

   ; Calcular colScreen = 8 + (posCursor % DIMMATRIX) * 4
   mov rax, rdx             ; rax = posCursor % DIMMATRIX
   shl rax, 2               ; rax = (posCursor % DIMMATRIX) * 4
   add rax, 8               ; rax = 8 + (posCursor % DIMMATRIX) * 4

   ; Preparar parámetros para gotoxyP2
   mov rdi, rbx             ; rdi = rowScreen (primer parámetro)
   mov rsi, rax             ; rsi = colScreen (segundo parámetro)
   call gotoxyP2            ; Llamar a gotoxyP2 para posicionar el cursor

posCurBoardP2_end:
   ; Restaurar registros
   pop rdx                  ; Restaurar rdx
   pop rbx                  ; Restaurar rbx
   pop rax                  ; Restaurar rax
   mov rsp, rbp             ; Restaurar rsp
   pop rbp                  ; Restaurar rbp
   ret                      ; Retornar
;funcional
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Muestra en el tablero el símbolo del jugador (stoneSymbol) de la ficha
; jugada en la posición actual del cursor (posCursor).
; Obtener el símbolo que se debe mostrar de la matriz (mBoard)
; [mBoart+posCursor] = mBoard[row][col], donde row=posCursor/DIMMATRIX
; y col=posCursor%DIMMATRIX.
; Mostrar el símbolo (stoneSymbol) llamando a la subrutina printchP2.
;  
; Variables globales utilizadas:	
; Ninguno.
; 
; Parámetros de entrada:
; (posCursor):rdi(rdi): subrutina
; (mBoard)   :rsi(rsi): Dirección de la matriz donde guardamos las fichas introducidas.
; 
; Parámetros de salida: 
; Ninguno.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;EX 2
showStonePosP2:
   push rbp
   mov  rbp, rsp
   push rax
   push rbx

   ; rdi = posCursor, rsi = mBoard
   mov rbx, rsi
   mov rax, rdi
   mov al, [rbx + rax]  ; cargar símbolo mBoard[posCursor] en al
   mov dil, al
   call printchP2

   pop rbx
   pop rax
   mov rsp, rbp
   pop rbp
   ret
 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Actualizar la posición donde está el cursor (posCursor) según
; el carácter (charac) leído de teclado:
; 'i' Arriba, 'j' Izquierda, 'k' Abajo, 'l' Derecha.
; Actualizar la posición del cursor (posCursor)
; controlando que no salga del tablero [0..(DIMMATRX*DIMMATRIX-1)].
; (row) [0..(DIMMATRX-1] - (col) [0..(DIMMATRX-1]
; Devolver el valor actualizado de (posCursor).
;  
; Variables globales utilizadas:	
; Ninguna.
; 
; Parámetros de entrada: 
; (posCursor):rdi(rdi): Posición del cursor dentro de la matriz mBoard.
; (charac)   :rsi(sil): Carácter leído de teclado.
; 
; Parámetros de salida: 
; (posCursor):rax(rax): Posición del cursor dentro de la matriz mBoard.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;EX 3

global moveCursorP2
section .text

moveCursorP2:
    push rbp
    mov  rbp, rsp
    push rbx
    push rcx
    push rdx

    ; rdi = posCursor
    ; sil = charac

    ; Obtener fila y columna: row = rdi / 10, col = rdi % 10
    mov rax, rdi
    mov rbx, 10
    xor rdx, rdx
    div rbx            ; rax = row, rdx = col
    mov r8, rax        ; r8 = row
    mov r9, rdx        ; r9 = col

    ; Obtener charac en cl
    movzx rcx, sil     ; cl = charac

    ; Mover según la tecla presionada
    cmp cl, 'i'
    jne .check_j
    cmp r8, 0
    je .ret_original
    dec r8
    jmp .recalculate

.check_j:
    cmp cl, 'j'
    jne .check_k
    cmp r9, 0
    je .ret_original
    dec r9
    jmp .recalculate

.check_k:
    cmp cl, 'k'
    jne .check_l
    cmp r8, 9
    jge .ret_original
    inc r8
    jmp .recalculate

.check_l:
    cmp cl, 'l'
    jne .ret_original
    cmp r9, 9
    jge .ret_original
    inc r9

.recalculate:
    ; posCursor = row * 10 + col
    mov rax, r8
    imul rax, 10
    add rax, r9
    jmp .done

.ret_original:
    mov rax, rdi

.done:
    pop rdx
    pop rcx
    pop rbx
    mov rsp, rbp
    pop rbp
    ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Contar cuántas casillas están ocupadas (!=' ') alrededor de la
; posición actual del cursor (posCursor) en la matriz (mBoard), en todas
; las direcciones, indicadas en el vector (dirLines) con el valor que
; debemos modificar la posición actual para seguir aquella dirección.
; dirección: 0           1             2             3             4           5        6   7
;    { -DIMMATRIX-1, +DIMMATRIX+1, +DIMMATRIX-1, -DIMMATRIX+1, -DIMMATRIX, +DIMMATRIX, -1, +1}
;       superior      inferior      inferior      superior      superior    inferior  izq. derecha
;       izquierda     derecha       izquierda     derecha
; Mientras no hayamos mirado todas las direcciones (dir<8) hacer:
;   Obtenemos la posición de la casilla que queremos mirar (nextPos) con
;   el incremento indicado en la matriz (dirLines) de la dirección que
;   estamos mirando (dir) (nextPos = posCursor + dirLines[dir]).
;   Si la fila y la columna están dentro del tablero
;   ((nextRow > 0) && (nextRow < DIMMATRIX)) ((nextCol>=0 && (col-nextCol) >= -1) && ((col-nextCol) <= 1))
;   y aquella casilla de la matriz (mBoard) no está vacía (!=' ')
;     Incrementamos (neighbors++).
;   Incrementamos (decir) para mirar en la siguiente dirección.
; Devolvemos (neighbors) para indicar cuántas casillas ocupadas
; hay alrededor de la posición actual.
; 
; Variables globales utilizadas:	
; Ninguna.
; 
; Parámetros de entrada: 
; (posCursor):rdi(rdi): Posición del cursor dentro de la matriz mBoard.
; (mBoard)   :rsi(rsi): Dirección de la matriz donde guardamos las fichas introducidas.
; (dirLines) :rdx(rdx): Dirección de la matriz que indica el incremento que se tiene que hacer a la posición actual para seguir una dirección.
;  
; Parámetros de salida: 
; (neighbors):rax(eax): Indica cuantas casillas ocupadas hay alrededor de la posición actual.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;EX 4;
global checkAroundP2
section .text

checkAroundP2:
    push rbp
    mov rbp, rsp
    push rbx
    push r12
    push r13
    push r14
    push r15

    ; Guardamos parámetros
    mov r14, rdi       ; r14 = posCursor
    mov r15, rsi       ; r15 = mBoard
    mov r13, rdx       ; r13 = dirLines

    xor r12d, r12d     ; contador de vecinos = 0
    xor ecx, ecx       ; índice dirLines: 0 a 7

.loop_dirs:
    cmp ecx, 8
    jge .end_loop

    ; offset = dirLines[ecx]
    mov rbx, [r13 + rcx*8]
    add rbx, r14           ; nextPos = posCursor + offset

    ; nextRow = nextPos / 10
    ; nextCol = nextPos % 10
    mov rdx, 0
    mov rax, rbx
    mov r8, 10
    div r8
    mov r9, rax            ; r9 = nextRow
    mov r10, rdx           ; r10 = nextCol

    ; Validar 0 ≤ nextRow, nextCol < 10
    cmp r9, 0
    jl .skip
    cmp r9, 10
    jge .skip
    cmp r10, 0
    jl .skip
    cmp r10, 10
    jge .skip

    ; Acceso: mBoard[nextRow][nextCol]
    mov rax, r9
    imul rax, 10
    add rax, r10
    movzx ebx, byte [r15 + rax]
    cmp bl, ' '
    je .skip

    inc r12d              ; vecinos++

.skip:
    inc ecx
    jmp .loop_dirs

.end_loop:
    mov eax, r12d         ; retorno = número de vecinos

    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx
    pop rbp
    ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Insertar la ficha (stoneSymbol) del jugador, indicado por la
; variable (state), en la posición donde está el cursor (posCursor)
; en la matriz (mBoard), si la posición está vacía (' ') y
; las casillas de alrededor no están vacías.
; Si en la posición actual del cursor (posCursor) no hay ninguna ficha
; (mBoard[row][col]==' ') miramos si las casillas de alrededor de la posición
; actual (posCursor) no están vacías llamando a la subrutina checkAroundP2().
; Si podemos introducir la ficha (neighbors > 0):
;   Si el estado del juego es (state==1) el (stoneSymbol = STONESYMBOLPLAYER1),
;   si el estado del juego es (state==2) el (stoneSymbol = STONESYMBOLPLAYER2).
;   Poner el símbolo (stoneSymbol) en la matriz (mBoard) en la posición
;   donde está el cursor (posCursor).
;   Cambiamos de jugador, de jugador 1 a jugador 2 y de jugador 2 a judador 1
;   (state = 3 - state).
; Retornamos estado del juego.
; 
; Variables globales utilizadas:	
; Ninguna.
; 
; Parámetros de entrada: 
; (posCursor):rdi(rdi): Posición del cursor dentro de la matriz mBoard.
; (mBoard)   :rsi(rsi): Dirección de la matriz donde guardamos las fichas introducidas.
; (dirLines) :rdx(rdx): Dirección de la matriz que indica el incremento que se tiene que hacer a la posición actual para seguir una dirección.
; (state)    :rcx(cx) : Estado del juego.
; 
; Parámetros de salida: 
; (state) :rax(ax): Estat del joc.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;EX5
global insertStoneP2
%define STONE_X 'X'
%define STONE_O 'O'
%define DIMMATRIX 10

section .text

insertStoneP2:
    push rbp
    mov rbp, rsp
    push rbx
    push r12
    push r13
    push r14
    push r15

    ; rdi = posCursor
    ; rsi = mBoard
    ; rdx = dirLines
    ; rcx = state

    mov r12, rdi       ; posCursor
    mov r13, rsi       ; mBoard
    mov r14, rdx       ; dirLines
    mov r15d, ecx      ; state

    ; leer mBoard[posCursor]
    movzx eax, byte [r13 + r12]
    cmp al, ' '
    jne .return_state

    ; llamar a checkAroundP2(posCursor, mBoard, dirLines)
    mov rdi, r12
    mov rsi, r13
    mov rdx, r14
    call checkAroundP2
    cmp eax, 0
    jle .return_state

    ; insertar ficha según jugador
    cmp r15d, 1
    je .put_x
    mov byte [r13 + r12], STONE_O
    jmp .change_turn

.put_x:
    mov byte [r13 + r12], STONE_X

.change_turn:
    mov eax, 3
    sub eax, r15d
    jmp .done

.return_state:
    mov eax, r15d

.done:
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx
    mov rsp, rbp
    pop rbp
    ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Comprobar si la ficha introducida, en la posición (posCursor)
; de la matriz (mBoard), hace 5 en raya (fiveINaRow=1) en alguna
; dirección: diagonal, horizontal o vertical, indicada con el valor
; que debemos modificar la posición actual para seguir aquella dirección
; en el vector (dirLines).
; dirección: 0             1             2             3            4           5         6   7
;    { -DIMMATRIX-1, +DIMMATRIX+1, +DIMMATRIX-1, -DIMMATRIX+1, -DIMMATRIX, +DIMMATRIX,   -1, +1 }
;      ( izquierda / derecha )     ( izquierda / derecha )   ( izquierda / derecha )  (izq./der.) //(leftright: izq.=0, der.=1)
;             diagonal 1                  diagonal 2                 vertical          horizontal
; La linea de 5 (stonesINaRow==5), respecto de la posición actual (X)
; puede estar, para cada dirección: a la izquierda XXXX(X), a la derecha (X)XXXX
; o en ambos lados X(X)XXX o XX(X)XX o XXX(X)X.
; Obtenemos el símbolo de la posición actual del cursor (stoneSymbol = mBoard[row][col]).
; Mientras no hacemos 5 en raya (fiveINaRow==0) y no hayamos mirado todas las direcciones (dir<8) hacer:
;   Si miramos la izquierda de la posición actual (leftright==0)
;     contamos 1 ficha (stonesINaRow=1)(la que acabamos de poner).
;     si estamos mirando a la derecha (leftright==1) no modificamos
;     (stonesINaRow) y seguimos contando fichas iguales.
;   Mientras podamos seguir buscando en esa dirección (exit==0)
;     Obtenemos la posición de la casilla que queremos mirar (nextPos)
;     con el incremento indicado en la matriz (dirLines) de la dirección
;     que estamos mirando (decir).
;     (nextPos = nextPos + dirLines[dir])
;     Si la fila o la columna está fuera del tablero
;     ((nextRow < 0) || (nextRow >= DIMMATRIX)) y
;     ((nextCol < 0) || ((col-nextCol) < -1) || ((col-nextCol) > 1))
;     dejamos de buscar en esa dirección (exit=1).
;     Si está dentro del tablero, miramos si el símbolo que hay en aquella
;     casilla (mBoard[nextRow][nextCol]) es el matex símbolo
;     que la casilla inicial (stoneSymbol),
;     si es el mismo símbolo incrementamos (stonesINaRow),
;     si no es lo mismo, dejamos de buscar en esa dirección (exit=1).
;   Si estábamos buscando hacia la izquierda (leftright==0) pasaremos
;   en busca a la derecha (leftright=1),
;   sino, pasaremos a busca a la izquierda (leftright=0),
;   en una nueva dirección (dir++) de la matriz (dirLines).
;   Si (stonesINaRow==5), tenemos un 5 en raya,
;   lo indicamos poniendo (fiveINaRow=1).
; Devolvemos (fiveINaRow) para indicar si hemos encontrado un 5 en raya o no.
; 
; Variables globales utilizadas:	
; Ninguna.
; 
; Parámetros de entrada: 
; (posCursor):rdi(rdi): Posición del cursor dentro de la matriz mBoard.
; (mBoard)   :rsi(rsi): Dirección de la matriz donde guardamos las fichas introducidas.
; (dirLines) :rdx(rdx): Dirección de la matriz que indica el incremento que se tiene que hacer a la posición actual para seguir una dirección.
;  
; Parámetros de salida: 
; (fiveINaRow):rax(al): Indica si hemos hecho 5 en raya (1) o no (0).
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;EX 6
checkRowP2:
   push rbp
   mov  rbp, rsp
   ;guardar el estado de los registros que se modifican en esta 
   ;subrutina y que no se utilizan para retornar valores.
   
   
   
   checkRowP2__end:  
   ;restaurar el estado de los registros que se han guardado en la pila.
   		
   mov rsp, rbp
   pop rbp
   ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Verifica si el jugador que ha introducido la última ficha ha realizado
; 5 en raya o si el tablero está lleno y no se puede seguir jugando.
; Llamando a la subrutina checkRowP2 comprobar si con la ficha
; introducida, en la posición (posCursor) de la matriz (mBoard),
; hace 5 en raya en alguna dirección: diagonal, horizontal o vertical
; y actualiza la variable (fiveINaRow: 1, hay 5 en raya, 0, no).
; Si hay 5 en raya (fourINaRow==1) incrementamos (state) en 2 para
; indicar que el jugador que ha introducido la ficha gana.
; Sino, recorremos todo el tablero (mBoard) para mirar si está lleno,
; DIMMATRIX*DIMMATRIX posiciones.
;   El tablero está lleno si todas las posiciones de la matriz (mBoard)
;   son diferentes de ' '.
;   Si está lleno, hemos recorrido todo el tablero y no hay ningún espacio,
;   pondremos (state=5) para indicar que el tablero está lleno
;   y no se puede seguir jugando.
; Devolver el estado del juego.
; 
; Variables globales utilizadas:	
; Ninguna.
; 
; Parámetros de entrada: 
; (posCursor):rdi(rdi): Posición del cursor dentro de la matriz mBoard.
; (mBoard)   :rsi(rsi): Dirección de la matriz donde guardamos las fichas introducidas.
; (dirLines) :rdx(rdx): Dirección de la matriz que indica el incremento que se tiene que hacer a la posición actual para seguir una dirección.
; (state)    :rcx(cx) : Estado del juego.
;  
; Parámetros de salida: 
; (state)    :rax(ax) : Estado del juego.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
checkEndP2:
   push rbp
   mov  rbp, rsp
   ;guardar el estado de los registros que se modifican en esta 
   ;subrutina y que no se utilizan para retornar valores.
   
   
   
   checkEndP2_end:
   ;restaurar el estado de los registros que se han guardado en la pila.
   		
   mov rsp, rbp
   pop rbp
   ret

  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Mostrar un mensaje en la parte inferior del tablero según el 
; valor de la variable (state).
; (state) 0: Se ha presionado ESC para salir 
;         1: Juega el jugador 1.
;         2: Juega el jugador 2.
;         3: El jugador 1 ha hecho 5 en línea.
;         4: El jugador 2 ha hecho 5 en línea.
;         5: El tablero está lleno. Empate.
; 
; Variables globales utilizadas:	
; Ninguna.
; 
; Parámetros de entrada: 
; (state): rdi(di): Estat del joc.
; 
; Parámetros de salida: 
; Ninguno
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
printMessageP2:
   push rbp
   mov  rbp, rsp
   ;guardamos el estado de los registros del procesador porque
   ;las funciones de C no mantienen el estado de los registros.
   push rax
   push rbx
   push rcx
   push rdx
   push rsi
   push rdi
   push r8
   push r9
   push r10
   push r11
   push r12
   push r13
   push r14
   push r15

   ; Cuando llamamos a la función printMessageP2_C(int state) desde ensamblador, 
   ; el parámetro (state) debe pasarse por el registro rdi(di).
   call printMessageP2_C
 
   ;restaurar el estado de los registros que se han guardado en la pila.
   pop r15
   pop r14
   pop r13
   pop r12
   pop r11
   pop r10
   pop r9
   pop r8
   pop rdi
   pop rsi
   pop rdx
   pop rcx
   pop rbx
   pop rax

   mov rsp, rbp
   pop rbp
   ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Subrutina principal del juego
; Muestra el tablero de juego y deja hacer las jugadas de los 2 jugadores,
; alternativamente, hasta que uno de los dos jugadores pone 5 fichas en raya
; o el tablero queda lleno y nadie ha hecho un 5 en raya.
;
; Pseudo-código:
; Mostrar un mensaje según el estado del juego (state) llamando a
; la subrutina printMessageP2.
; Mientras estén jugando el jugador 1 (state=1) o el jugador 2 (state=2) hacer:
;   Posicionar el cursor en el tablero en la posición indicada por la variable
;   (posCursor) llamando a la subrutina posCurBoardP2.
;   Leer una tecla llamando a la subrutina getchP2.
;   Si la tecla leída es 'i', 'j', 'k' o 'l' mover el cursor sin
;   salir del tablero llamando a la subrutina moveCursorP2.
;   Si la tecla leída es ' ' comprobar si la las casillas que hay
;   alrededor de la posición actual hay alguna ficha llamando
;   a la subrutina checkAroundP2.
;     Si hay alguna posición ocupada (neighbors>0)
;       Insertar la ficha en el tablero (mBoard) en la posición actual del
;       cursor (posCursor) llamando a la subrutina insertStoneP2.
;       Si se ha introducido la ficha, (state != newState)
;         Mostrar la ficha que se ha insertado en el tablero (mBoard) en la posición
;         actual del tablero (posCursor) llamando a la subrutina showStonePosP2.
;         Verificar si se ha hecho 5 en raya o el tablero está lleno
;         llamando la subrutina checkEndP2.
;       Si no se ha hecho 5 en raya o el tablero está lleno (state <= 2)
;       actualizar el estado del juego (state = newState).
;   Si la tecla es ESC(ASCII 27) poner (state=0) para indicarlo.
; Mostrar un mensaje según el estado del juego (state) llamando
; a la subrutina printMessageP2.
; Se termina el juego.
; 
; Variables globales utilizadas:	
; Ninguna.
; 
; Parámetros de entrada: 
; (mBoard)  :rdi(rdi): Dirección de la matriz donde guardamos las fichas introducidas.
; (dirLines):rsi(rsi): Dirección de la matriz que indica el incremento que se tiene que hacer a la posición actual para seguir una dirección.
; 
; Parámetros de salida: 
; Ninguno
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
playP2:
   push rbp
   mov  rbp, rsp
   ;guardar el estado de los registros que se modifican en esta 
   ;subrutina y que no se utilizan para retornar valores.	
   push rax
   push rbx
   push rcx
   push rdx
   push rsi
   push rdi
   push r8
   push r9
   push r10
   push r11

   mov r8, rdi                ;(mBoard)
   mov r9, rsi                ;(dirLines)
   mov r10, 44                ;long  posCursor = 44; 
   mov bx, 1                  ;short state = 1; // Estado del juego
                              ;// 0: Se ha presionado ESC para salir 
                              ;// 1: Juega el jugador 1.
                              ;// 2: Juega el jugador 2.
                              ;// 3: El jugador 1 ha hecho 5 en línea.
                              ;// 4: El jugador 2 ha hecho 5 en línea.
                              ;// 5: El tablero está lleno. Empate.
   mov  r11w, bx              ;newState = state;                   
   mov  di, bx
   call printMessageP2        ;printMessageP2_C(state);
   
   playP2_while:
   cmp bx, 1                  ;while (state == 1 
   je  playP2_loop
     cmp bx, 2                ;|| state ==2  ) {
     jne playP2_end
     playP2_loop:
       mov  rdi, r10
       call posCurBoardP2     ;posCurBoardP2_C(posCursor);
       call getchP2           ;charac = getchP2_C();   
       playP2_move:
       cmp al, 'i'            ;if (charac >= 'i'  
       jl  playP2_insert
         cmp al, 'l'          ;&& charac <= 'l') {
         jg playP2_insert
           mov rdi, r10
           mov sil, al
           call moveCursorP2  ;posCursor = moveCursorP2_C(posCursor, charac);
           mov r10, rax
       jmp playP2_endwhile;}
       playP2_insert:      
       cmp al, ' '            ;if (charac == ' ' ) {
       jne playP2_esc
         mov rdi, r10
         mov rsi, r8
         mov rdx, r9
         call checkAroundP2   ;neighbors  = checkAroundP2_C(posCursor, mBoard, dirLines);
         cmp eax, 0           ;if (neighbors  > 0) {
         jle playP2_noneighbours
           mov  cx, bx
           call insertStoneP2  ;newState = insertStoneP2_C(posCursor, mBoard, dirLines, state);
           mov  r11w, ax
           cmp bx, r11w       ;if(state != newState){ //new stone inserted
           je  playP2_notinserted
             call showStonePosP2;showStonePosP2_C(posCursor, mBoard);
             call checkEndP2  ;state = checkEndP2_C(posCursor, mBoard, dirLines, state);
             mov bx, ax
           playP2_notinserted:;}
           cmp bx, 2          ;if (state <= 2) 
           jg  playP2_newstate
             mov bx, r11w     ;state = newState;
             playP2_newstate: ;}
         playP2_noneighbours: ;}
       jmp playP2_endwhile    
       playP2_esc:
       cmp al, 27             ;if (charac == 27) {
       jne playP2_noesc
         mov bx, 0           ;state = 0;
       playP2_noesc:         ;} 
     playP2_endwhile:        ;}
     mov  di, bx
     call printMessageP2     ;printMessageP2_C(state);  
     jmp playP2_while 
   playP2_end: 
   ;restaurar el estado de los registros que se han guardado en la pila.
   pop r10
   pop r9
   pop r8
   pop rdi
   pop rsi
   pop rdx
   pop rcx
   pop rbx
   pop rax

   mov rsp, rbp
   pop rbp
   ret
