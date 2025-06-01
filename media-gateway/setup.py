from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

with open("requirements.txt", "r", encoding="utf-8") as fh:
    requirements = [line.strip() for line in fh if line.strip() and not line.startswith("#")]

setup(
    name="open-teleop-media-gateway",
    version="0.1.0",
    author="Open Teleop Contributors",
    author_email="",
    description="High-performance media gateway for Open Teleop platform",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/open-teleop/open-teleop",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Multimedia :: Video :: Capture",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: System :: Hardware",
    ],
    python_requires=">=3.8",
    install_requires=requirements,
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "pytest-asyncio>=0.21.0",
            "pytest-mock>=3.10.0",
            "mypy>=1.0.0",
            "black>=22.0.0",
            "flake8>=5.0.0",
            "isort>=5.10.0",
        ],
        "profiling": [
            "line_profiler>=4.0.0",
            "memory_profiler>=0.60.0",
            "py-spy>=0.3.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "media-gateway=media_gateway.main:main",
        ],
    },
    include_package_data=True,
    package_data={
        "media_gateway": [
            "config/*.yaml",
            "schemas/*.fbs",
        ],
    },
    project_urls={
        "Bug Reports": "https://github.com/open-teleop/open-teleop/issues",
        "Source": "https://github.com/open-teleop/open-teleop",
        "Documentation": "https://github.com/open-teleop/open-teleop/docs",
    },
) 