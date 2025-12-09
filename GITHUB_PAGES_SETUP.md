# GitHub Pages Setup for Docusaurus

## Configuration Details

The Docusaurus site is configured for GitHub Pages deployment with the following settings:

### Docusaurus Configuration (`docusaurus.config.ts`)
- `organizationName`: `SyedFarooqAlii`
- `projectName`: `Physical-AI-Humanoid-Robotics-Textbook`
- `url`: `https://SyedFarooqAlii.github.io`
- `baseUrl`: `/Physical-AI-Humanoid-Robotics-Textbook/`

### GitHub Actions Workflow
- Uses the `actions/deploy-pages` action
- Builds the site in the `humanoid-robotics-book` subdirectory
- Deploys to GitHub Pages via the `gh-pages` branch (created automatically)

### GitHub Pages Source
- The GitHub Actions workflow automatically creates and updates the `gh-pages` branch
- GitHub Pages should be configured to use the `gh-pages` branch as the source

## Important Notes

1. When Docusaurus builds with a `baseUrl` like `/Physical-AI-Humanoid-Robotics-Textbook/`, all paths are prefixed with this base URL
2. The site will be accessible at: `https://SyedFarooqAlii.github.io/Physical-AI-Humanoid-Robotics-Textbook/`
3. The GitHub Actions workflow handles the deployment automatically on pushes to the `main` branch

## Troubleshooting

If GitHub Pages still shows README.md:
1. Check that GitHub Pages source is set to "Deploy from a branch" or uses GitHub Actions
2. Wait for the GitHub Actions workflow to complete after the latest push
3. The first deployment may take a few minutes to propagate