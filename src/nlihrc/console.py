"""
CLI entrypoints for nlihrc
NOTE: ROS MASTER NODE MUST BE RUNNING TO USE THIS APPLICATION!
"""
from pathlib import Path

import click
import toml
from nlihrc import __version__
from nlihrc.main import main_speech, main_robot


@click.group()
@click.version_option(version=__version__)
@click.argument("config-path", type=click.Path(exists=True, file_okay=True, dir_okay=False))
@click.pass_context
def nlihrc_cli(ctx, config_path: Path,) -> None:
    """Main Entrypoint"""
    ctx.ensure_object(dict)
    config = toml.load(config_path)
    ctx.obj['CONFIG'] = config


@nlihrc_cli.command()
@click.pass_context
def speech(ctx):
    """Run speech server"""
    config = ctx.obj['CONFIG']
    click.echo("Running Speech only server...")
    main_speech(config)


@nlihrc_cli.command()
@click.pass_context
def robot(ctx):
    """Run robot server"""
    config = ctx.obj['CONFIG']
    click.echo("Running Robot only server...")

    main_robot(config)
