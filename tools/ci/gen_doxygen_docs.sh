#!/bin/bash
################################################################################
##### Setup this script and get the current gh-pages branch.               #####
echo 'Setting up the script...'
# Exit with nonzero exit code if anything fails
set -e

# Get the current gh-pages branch
cd "$TRAVIS_BUILD_DIR"
git clone --recursive -b gh-pages "https://$GH_REPO_REF" gh-pages
cd gh-pages

# Need to create a .nojekyll file to allow filenames starting with an underscore
# to be seen on the gh-pages site. Therefore creating an empty .nojekyll file.
# Presumably this is only needed when the SHORT_NAMES option in Doxygen is set
# to NO, which it is by default. So creating the file just in case.
echo "" > .nojekyll

################################################################################
##### Generate the Doxygen code documentation and log the output.          #####
echo 'Generating Doxygen code documentation...'
cd "$TRAVIS_BUILD_DIR/$DOXYFILE_PATH"
# Redirect both stderr and stdout to the log file AND the console.
doxygen "$DOXYFILE" 2>&1 | tee "$TRAVIS_BUILD_DIR/gh-pages/doxygen.log"

################################################################################
##### Upload the documentation to the gh-pages branch of the repository.   #####
# Only upload if Doxygen successfully created the documentation.
# Check this by verifying that the html directory and the file html/index.html
# both exist. This is a good indication that Doxygen did it's work.
if [ -d "html" ] && [ -f "html/index.html" ]; then
    echo 'Doxygen successfully generated the docs.. copying to gh-pages..'
    rm -rf "$TRAVIS_BUILD_DIR/gh-pages/html"
    cp -rf html "$TRAVIS_BUILD_DIR/gh-pages/"
else
    echo '' >&2
    echo 'Warning: Doxygen failed to generate the docs.. no html build found!' >&2
    exit 1
fi