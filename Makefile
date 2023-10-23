TARGET ?= default

# Define the default target
default: 
	python3 robot.py

# Rule to generate a target of the same name
%: 
	python3 robot.py $@

clean:
	find . -iname '*.pyc' -delete
	find . -iname '__pycache__' -delete

.PHONY: default clean
