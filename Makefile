TARGET ?= default

# Define the default target
default: 
	python3 robot.py

download:
	python3 -m robotpy_installer download-python
	python3 -m robotpy_installer download robotpy[all]

install:
	python3 -m robotpy_installer install-python
	python3 -m robotpy_installer install robotpy[all]

# Rule to generate a target of the same name
%: 
	python3 robot.py $@

clean:
	find . -iname '*.pyc' -delete
	find . -iname '__pycache__' -delete

.PHONY: default clean download install
