function [h] = plot_wrapper(x, y, lw, ls, c, xl, yl)
    h = plot(x, y, 'LineWidth', lw, 'Color', c, 'LineStyle', ls);
    ylabel(yl, 'interpreter', 'latex','FontSize', 17);
    xlabel(xl, 'interpreter', 'latex','FontSize', 17);
end

