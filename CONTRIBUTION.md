# Contribution Guidelines

Follow this guide to start contributing to RAIN

## Configuring

* Fork the repository to your github account by pressing the 'Fork' button on the top right corner of the screen when you open the repo

* Clone your fork to your computer using :
	
	```
	git clone git@github.com:your-github-username/RAIN.git
	```

* Set the remotes :

	```bash
	git remote add origin git@github.com:your-github-username/RAIN.git
	git remote add upstream git@github.com:ERC-BPGC/RAIN.git
	```

* Updating the forks:
	```bash
	git fetch upstream
	```

<!-- * Installing dependencies:
	```bash
	python2 -m pip install -r requirements.txt
	python3 -m pip install -r requirements.txt
	python3 -m pip install black 
	``` -->

## Contibuting
* Creating a new branch for your contributions:
	```bash
	git checkout -b name-of-your-bugfix-or-feature
	```

* Remember to update your local repo before starting the work everytime by using :
	
	```
	git pull upstream main
	```

* You can create a new branch where you will make the changes using :
	```
	git checkout -b #branch_name
	```


This will push the changes to the forked remote repo. Once this is done, you can open 'Pull Request' (PR) to the repo which will then be reviwed and merged after making some changes (if any)


If you add any new code in the package, use [Google Style](https://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_google.html) to write the documentation.


* To format code use the following:
	```bash
	isort .
	black .
	flake8 .
	```

* After making the final changes, you can push the new changes using :
	```bash
	git add .
	git commit -m 'comments'
	git push origin #branch_name
	```