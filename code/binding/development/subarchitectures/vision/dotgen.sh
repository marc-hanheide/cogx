rm dotfiles/*.pdf
find ./dotfiles -name *.dot -exec dot -Tpdf -O{} {} \;
rm dotfiles/*.dot
