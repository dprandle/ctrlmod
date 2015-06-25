(( nil . ((compile-command ."make -C /home/dprandle/Documents/code/ctrlmod/build")
		  (eval . (global-ede-mode 1))
		  (eval . (add-to-list 'achead:include-directories '"~/Documents/code/ctrmod/include"))
		  (eval . (ede-cpp-root-project "ctrlmod" :file "/home/dprandle/Documents/code/ctrlmod/CMakeLists.txt"
										:include-path '( "/include")))
		  (eval . (message "File loaded with EDE")))))
