# Qulog/Teleor
A docker package for QuLog/Teleor: QuLog is a higher-order logic/functional/(string processing)/(imperative rule) language, the Teleor extension allows for the consultation of TeleoR procedures.  

- QuLog is a higher-order logic/functional/string processing language with an imperative rule language sitting on top, defining actions. QuLog's action rules are used to program multi-threaded communicating agent behaviour. Its declarative subset is used for the agent's belief store. The language is flexibly typed and allows a combination of compile time and run-time type checking.
- The Teleor extension allows program files to be consulted containing TeleoR procedures as well as QuLog rules. This extension includes a generic agent shell that can be launched to execute calls to TeleoR procedures as tasks. It can be configured by including specially named QuLog action procedures and relations in your program file.

The authors of Qulog & its Teleor extension are:
- Peter Robinson (pjr@itee.uq.edu.au)
- Keith Clark  

for more information on  Qulog & its Teleor extension, please visit: [http://staff.itee.uq.edu.au/pjr/HomePages/QulogHome.html](http://staff.itee.uq.edu.au/pjr/HomePages/QulogHome.html)

# Package Installation & Usage:
- pull the docker image:
  ```sh
  docker pull docker.pkg.github.com/alyshmahell/qulog-teleor/qulog-teleor:0.7
  ```
- retag it for easier access:
  ```sh
  docker tag docker.pkg.github.com/alyshmahell/qulog-teleor/qulog-teleor:0.7 qulog-teleor:0.7
  ```
- run the image:
  ```sh
  docker run -i -t qulog-teleor:0.7
  ````
- running qulog:
  ```sh
  qulog
  ```

# Package Information:
- This package has Ubuntu 18.04 as a base
- Qulog/Teleor 0.7 is installed within it.