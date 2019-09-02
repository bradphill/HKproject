# Git info
Liten info för hur vi använder git. Försöker på en liten step-by-step lista

# Issue
För varje issue skapar vi en ny branch att jobba i. Detta för inte sabba huvudkoden när vi arbetar i olika filer. 
Så länge vi inte ändrar samma grej i samma fil bör det vara lugnt.
När ett issue är klar och kompilerar så mergas den in i master

# Kommandon i terminal för git

1. Git pull för senaste versionen av koden från. 
```bash
cd /user/desktop/projektmapp
git pull origin master
```

2. Git checkout för skapa en ny branch för ditt issue. -b för ny branch
```bash
git checkout -b myBranch 
```

3. Git status för se filer du ändrat i och vad som är "addat" till ditt repo/ branch.
rött = inte addat
grönt = addat

```bash
git status
git add . (punkten för lägga till alla filer, annars skriv vilken fil du vill lägga till)
```

4. Git commit, lägger till ett meddelande med info om vad ändringen gör.
```bash
git commit -m "info meddelande"
```

5.Git push, för trycka upp det på git
```bash
git push origin myBranch
```

# Sätta upp git för första gången på din dator. 
cd för change directory till den mapp du vill att projektet ska hamna i
```bash
cd /user/desktop/projektmapp
```
git clone för ladda ner senaste versionen, url addressen från det repo du vill dra ner
```bash
git clone https://github.com/bradphill/HKproject.git
```
