## Interagindo por CLI

No **terminal 1**, execute

> `$ rostopic`

para subir a aplicação *master* e depois aperte `Ctrl+Z` para deixar essa aplicação em segundo plano. Depois, use

> `$ rostopic list`

e veja que apenas os tópicos `/rosout`, referentes ao *master*, foram criados.

No **terminal 2** deixe a listagem rodando de forma periódica com

> `$ watch -n 1 rostopic list`

que fará com que a tela seja atualizada a cada 1 segundo.

No **terminal 3** crie um novo tópico de nome */sensor/value* que publique o número 42 com frequência de 2 Hz, com o comando

> `$ rostopic pub -r 2 /sensor/value std_msgs/Int32 "data: 42"`

Repare no **terminal 2** que a lista de tópicos foi atualizada. No **terminal 1** digite

> `$ rostopic echo /sensor/value`

para que o conteúdo publicado seja exibido no terminal. Perceba que ao cancelar o comando de publicação do **terminal 3** (`Ctrl+C`) tanto o **terminal 1** para de receber valores como o **terminal 2** não contém mais o tópico em questão.

Agora cancele o comando do **terminal 2** e troque para

> `$ watch -n 1 rostopic info /sensor/value`

assim, será exibido as informações do tipo de mensagem, que nó está publicando e que nó está consumindo esses valores.

Inicialmente será exibido que não há um tópico com esse nome, mas ao reexecutar o comando `rostopic pub` no **terminal 3** logo a informação é atualizada. E ao se executar o `rostopic echo` no **terminal 1** também aparece um *subscriber* para esse tópico.

Para encerrar a aplicação *master* que está rodando em segundo plano no **terminal 1**, basta executar 

> `$ fg`

para trazê-la de volta ao primeiro plano e então pressionar `Ctrl+C` para interromper a execução.

<hr>
## Criando Pacotes

Um *workspace* do catkin contém sempre 3 pastas: `build`, `devel` e `src`. As duas primeiras são manipuladas automaticamente pelo framework e não devem ser mexidas.

Mude para a pasta de código-fonte com

> `$ cd catkin/ws/src`

e depois crie um pacote com

> `catkin_create_kg my_robot std_msgs rospy roscpp`

onde `my_robot` é o nome do pacote a ser criado e os demais termos são as APIs que queremos associar ao projeto (elas não precisam ser listadas aqui, mas isso evita de ter que adicioná-las depois em  arquivos de configuração).

Entre na pasta do pacote e abra o arquivo `packages.xml` para editar as informações sobre o pacote, caso queira disponibilizá-lo.

> `$ cd my_robot`

> `$ vim packages.xml`

Observe que dentro da pasta do pacote foram criados automaticamente um arquivo `CMake` e pastas `include` e `src`. Para efeitos de organização, será criada uma terceira pasta `script`, onde serão colocados os códigos em Python.

> `$ mkdir script`

<hr>
## Python API

Com o VScode, abra a pasta de *scripts* e crie um arquivo com o nome `sensor_node.py`, que corresponderá a um nó de sensor *publisher* dentro do ROS, e adicione na primeira linha o código

> `#!/usr/bin/env python`

para indicar que esse arquivo, ao ser chamado como um executável, é uma aplicação Python

Após criar o arquivo, é preciso dar permissão para que ele seja tratado como um executável, e não apenas chamando o interpretador da linguagem.

> `$ cd script`

> `$ sudo chmod +x sensor_node.py`

Para compilar o projeto, é preciso ir até a pasta do *workspace* e chamar o comando de *make* do catkin.

> `$ cd ../../..`

> `$ catkin_make`

Também é preciso executar o arquivo de `setup`, que irá criar as variáveis de ambiente do projeto.

> `$ source devel/setup.bash`

Por fim, para testar o nó criado, será iniciado a aplicação *master* novamente e, com o uso de dois terminais, será verificado a publicação dos tópicos da aplicação `sensor_node`.

No **terminal 1**:

> `$ roscore`

> `Ctrl+Z`

`$ rosrun my_robot sensor_node.py`

Em caso de edição no *script* em Python não é necessário reexecutar o `make`, uma vez que o *script* não é compilado.

Para completar, adicione um arquivo `app_node.py` para atuar como um *subscriber*, e execute os passos anteriores (permissão de execução e comando `make`).

Ao ter o `roscore` e o *publisher* rodando, execute o *subscriber* em um terceiro terminal e acompanhe ele informando o recebimento de cada mensagem. Ao cancelar a execução do `sensor_node`, o `app_node` permanece em aguardo por mensagens, e volta a imprimir em tela uma vez que o *publisher* for reexecutado.

<hr>
## Aplicação: Controle de um AGV

**Tarefa**
- Implementar o controle de um AGV com 5 comandos (`forward`, `backward`, `right`, `left`, `stop`) e uma flag de colisão;
- O nó do motor é responsável por controlar a velocidade das rodas direita e esquerda pelos tópicos Float64 *motor/velocity/right* e *motor/velocity/left*;
- Os comandos de movimento do AGV são feitos pelo tópico String *control_msg*;
- O aviso de colisão iminente é feito pelo tópico Bool *collision/status*;
- O sinal de controle e colisão são enviador por CLI e recebidos pelo motor, enquanto que o sinal de velocidade é enviado pelo motor.

Foi criado um script `motor_node.py` e os seguintes comandos do terminal são executados para construir a interação desejada.

No **terminal 1** será checado os tópicos ativos e fito o acompanhamento do log de `motor_node`:

`$ roscore

`Ctrl+Z`

`$ rostopic list`

`$ rosrun my_robot motor_node`

No **terminal 2** será monitorado a velocidade da roda direita:

`$ rostopic echo /motor/velocity/right`

No **terminal 2** será monitorado o status do sensor de colisão:

`$ rostopic echo /collision/status`

No **terminal 4** serão publicadas mensagens usando os comandos como

`$ rostopic pub /control_msg std_msgs/String "data: left"`

`$ rostopic pub /collision/status std_msgs/Bool "data: True"`

<hr>
