import compileall
import py_compile
import os

if __name__ == '__main__':
	current_path = os.path.dirname(os.path.abspath(__file__))
	#print(current_path)
	compileall.compile_dir(current_path)
	f_list = os.listdir(current_path)
	for file in f_list:
		target_file = os.path.join(current_path, file)
		#if os.path.splitext(target_file)[1] == '.py':
			#os.remove(target_file)
