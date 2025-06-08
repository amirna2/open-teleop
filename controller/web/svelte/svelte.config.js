import adapter from '@sveltejs/adapter-static';

/** @type {import('@sveltejs/kit').Config} */
const config = {
	kit: {
		adapter: adapter({
			pages: '../static/dist',
			assets: '../static/dist',
			fallback: 'index.html',
			precompress: false,
			strict: true
		}),
		files: {
			assets: 'static'
		}
	}
};

export default config;