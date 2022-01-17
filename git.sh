git config --global user.name "LaiYanKai"
git config --global user.email "lai.yankai@gmail.com"
##### initial steps ######

#git init
#git add .
#git commit -m "first commit"
#git branch -M main
#git remote add origin https://<PAT>@github.com/LaiYanKai/rubiker.git
#git push -u origin main

git add *
git add .gitignore
git commit -m "$1"
git branch -M main
git push origin main
